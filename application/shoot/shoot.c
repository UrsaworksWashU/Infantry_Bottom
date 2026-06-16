#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "rm_referee.h"
#include <math.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

static referee_info_t *referee_data; // 裁判系统数据,用于读取枪口热量

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

// 连发目标射频(发/秒),仿中科大Target_Ammo_Shoot_Frequency,默认用此速度,可Ozone实时调
static float target_ammo_shoot_frequency = 10.0f;

volatile float dbg_loader_current_ref;
volatile float dbg_loader_current_measure;
volatile float dbg_loader_speed_ref;
volatile float dbg_loader_speed_measure;
volatile float dbg_loader_angle_ref;
volatile float dbg_loader_angle_measure;
volatile float dbg_friction_l_speed_ref;
volatile float dbg_friction_l_speed_measure;
volatile float dbg_friction_r_speed_ref;
volatile float dbg_friction_r_speed_measure;
volatile float friction_speed_ref;

// 拨盘调参：Ozone把 loader_tuning 设为1进入，0恢复正常
// loader_tune_loop: 0=电流环(直接给电流参考), 1=速度环(给速度参考)
// 调参时摩擦轮强制关闭，只动拨盘
volatile uint8_t loader_tuning    = 0;
volatile uint8_t loader_tune_loop = 1;       // 0=电流环, 1=速度环
volatile float   loader_cur_amp   = 2000.0f; // 电流环阶跃幅值(电流单位，同current PID MaxOut量纲)
volatile float   loader_cur_freq  = 1.0f;    // Hz，Ozone可实时改
volatile float   loader_spd_amp   = 300.0f;  // 速度环阶跃幅值 deg/s，Ozone可实时改
volatile float   loader_spd_freq  = 1.0f;    // Hz，Ozone可实时改

// 卡弹处理有限状态机, 仿中科大Class_FSM_Anti_Jamming
typedef enum { JAM_NORMAL = 0, JAM_SUSPECT, JAM_CONFIRM, JAM_PROCESSING } ShootJamStatus_e;
static ShootJamStatus_e jam_status = JAM_NORMAL;
static float jam_status_enter_time = 0; // ms, 进入当前状态的时刻
volatile uint8_t dbg_jam_status;        // Ozone观察卡弹状态

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 注意方向设置为发射的出弹方向
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 0, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 3, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    referee_data = RefereeGetInfo(); // 获取裁判系统数据指针(串口由UI/chassis初始化,此处仅取指针)

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/**
 * @brief 卡弹处理有限状态机, 仿中科大Class_FSM_Anti_Jamming::TIM_1ms_Calculate_PeriodElapsedCallback
 * @note  正常/嫌疑态由外部执行正常发射输出; 确认/处理态在此接管拨盘进行回拨
 */
static void ShootJamSetStatus(ShootJamStatus_e s)
{
    jam_status = s;
    jam_status_enter_time = DWT_GetTimeline_ms();
}

static void ShootJamFSM(void)
{
    float elapsed = DWT_GetTimeline_ms() - jam_status_enter_time;
    float cur = fabsf((float)loader->measure.real_current); // 绝对值, 不依赖拨盘方向
    switch (jam_status)
    {
    case JAM_NORMAL:
        // 大扭矩 -> 卡弹嫌疑状态
        if (cur >= JAM_CURRENT_THRESHOLD)
            ShootJamSetStatus(JAM_SUSPECT);
        break;
    case JAM_SUSPECT:
        if (elapsed >= JAM_SUSPECT_TIME_MS)
            ShootJamSetStatus(JAM_CONFIRM); // 长时间大扭矩 -> 卡弹反应状态
        else if (cur < JAM_CURRENT_THRESHOLD)
            ShootJamSetStatus(JAM_NORMAL); // 短时间大扭矩 -> 正常状态
        break;
    case JAM_CONFIRM:
        // 卡弹反应状态 -> 准备卡弹处理: 切到角度环回拨
        DJIMotorOuterLoop(loader, ANGLE_LOOP);
        DJIMotorSetRef(loader, loader->measure.total_angle - JAM_BACK_ANGLE);
        ShootJamSetStatus(JAM_PROCESSING);
        break;
    case JAM_PROCESSING:
        // 卡弹处理状态: 长时间回拨 -> 正常状态
        if (elapsed >= JAM_SOLVING_TIME_MS)
            ShootJamSetStatus(JAM_NORMAL);
        break;
    }
}

/* 发射输出逻辑(原ShootTask主体, 内容不变), 仅在卡弹状态机正常/嫌疑态时执行 */
static void ShootOutput()
{
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;                                                                    // 完成1发弹丸发射的时间
        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,根据枪口剩余热量(barrel heat余量)动态选择拨弹射频
    // 仿中科大开源步兵Booster_Control_Type_AUTO热量控制逻辑:
    //   余量充足时全速发射;余量进入缓冲区后在目标射频与可持续射频(冷却速率)之间线性过渡;余量过低时停火
    case LOAD_BURSTFIRE:
    {
        DJIMotorOuterLoop(loader, SPEED_LOOP);

        // 剩余热量 = 热量上限 - 当前17mm枪口热量 (= 中科大tmp_delta)
        float heat_remain = (float)(referee_data->GameRobotState.shooter_barrel_heat_limit - referee_data->PowerHeatData.shooter_17mm_barrel_heat);
        float now_rate;

        if (heat_remain >= HEAT_SLOWDOWN_THRESHOLD)
        {
            // 余量充足,全速发射
            now_rate = target_ammo_shoot_frequency;
        }
        else if (heat_remain >= HEAT_CEASEFIRE_THRESHOLD)
        {
            // 余量降低,在目标射频与可持续射频(冷却速率/每发热量)之间线性过渡
            // 余量=SLOWDOWN阈值时为target,余量=CEASEFIRE阈值时为冷却对应的可持续射频
            float sustain = (float)referee_data->GameRobotState.shooter_barrel_cooling_value / HEAT_PER_BULLET;
            now_rate = (target_ammo_shoot_frequency * (HEAT_CEASEFIRE_THRESHOLD - heat_remain) + sustain * (heat_remain - HEAT_SLOWDOWN_THRESHOLD)) / (HEAT_CEASEFIRE_THRESHOLD - HEAT_SLOWDOWN_THRESHOLD);
        }
        else
        {
            // 余量不足,停火
            now_rate = 0.0f;
        }

        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        DJIMotorSetRef(loader, now_rate * 360 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE);
        break;
    }
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    // 2026 ARCC Speed limits: 17mm 25m/s(45000-24m/s), 42mm 15m/s
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_25:
            DJIMotorSetRef(friction_l, 45000);
            DJIMotorSetRef(friction_r, 45000);
            break;
        default:
            DJIMotorSetRef(friction_l, 45000);
            DJIMotorSetRef(friction_r, 45000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // === 拨盘电流环/速度环阶跃调参 ===
    // Ozone把 loader_tuning 设为1进入，0恢复正常
    // 调电流环: loader_tune_loop=0；调速度环: loader_tune_loop=1
    // 用方波给阶跃，观察 dbg_loader_*_ref / dbg_loader_*_measure 跟随情况
    if (loader_tuning)
    {
        // 安全: 调拨盘时强制停摩擦轮
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorEnable(loader);

        float t = DWT_GetTimeline_s();

        if (loader_tune_loop == 0)
        {
            // 电流环: 外环切到CURRENT_LOOP, SetRef直接是电流参考
            DJIMotorOuterLoop(loader, CURRENT_LOOP);
            float half = 0.5f / loader_cur_freq;
            float ref  = (fmodf(t, 2.0f * half) < half) ? loader_cur_amp : -loader_cur_amp;
            DJIMotorSetRef(loader, ref);
        }
        else
        {
            // 速度环: 外环切到SPEED_LOOP, SetRef是速度参考(deg/s)
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            float half = 0.5f / loader_spd_freq;
            float ref  = (fmodf(t, 2.0f * half) < half) ? loader_spd_amp : -loader_spd_amp;
            DJIMotorSetRef(loader, ref);
        }

        dbg_loader_current_ref     = loader->motor_controller.current_PID.Ref;
        dbg_loader_current_measure = loader->motor_controller.current_PID.Measure;
        dbg_loader_speed_ref       = loader->motor_controller.speed_PID.Ref;
        dbg_loader_speed_measure   = loader->motor_controller.speed_PID.Measure;
        dbg_loader_angle_ref       = loader->motor_controller.angle_PID.Ref;
        dbg_loader_angle_measure   = loader->motor_controller.angle_PID.Measure;

        PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
        return;
    }

    // 急停时复位状态机, 保证 ShootOutput() 执行真正停机, FSM 不在急停期间劫持拨盘 (安全保护)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
        ShootJamSetStatus(JAM_NORMAL);

    // 卡弹处理状态机: 仿中科大Class_Booster
    // 正常/嫌疑态执行正常发射输出; 确认/处理态由FSM接管拨盘回拨, 摩擦轮保持上次转速
    ShootJamFSM();
    if (jam_status == JAM_NORMAL || jam_status == JAM_SUSPECT)
        ShootOutput();

    dbg_jam_status = (uint8_t)jam_status;
    dbg_loader_current_ref = loader->motor_controller.current_PID.Ref;
    dbg_loader_current_measure = loader->motor_controller.current_PID.Measure;
    dbg_loader_speed_ref = loader->motor_controller.speed_PID.Ref;
    dbg_loader_speed_measure = loader->motor_controller.speed_PID.Measure;
    dbg_loader_angle_ref = loader->motor_controller.angle_PID.Ref;
    dbg_loader_angle_measure = loader->motor_controller.angle_PID.Measure;
    dbg_friction_l_speed_ref = friction_l->motor_controller.speed_PID.Ref;
    dbg_friction_l_speed_measure = friction_l->motor_controller.speed_PID.Measure;
    dbg_friction_r_speed_ref = friction_r->motor_controller.speed_PID.Ref;
    dbg_friction_r_speed_measure = friction_r->motor_controller.speed_PID.Measure;

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}