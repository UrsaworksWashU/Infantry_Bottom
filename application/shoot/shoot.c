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
static float target_ammo_shoot_frequency = 20.0f;

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

// 发射机构状态由cmd经 shoot_cmd_recv.shoot_state 直接给定(robot_def.h: shoot_state_e),
// 仿中科大Booster_Control_Type, 单一状态机, shoot层不再维护内部状态
volatile uint8_t dbg_booster_type; // Ozone 观察发射机构状态(= shoot_cmd_recv.shoot_state)

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
                .Kp = 10, // 10
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

/* 摩擦轮(飞轮)输出: 仿中科大,在停火/单发/连发态常转,根据弹速设定转速 */
// 2026 ARCC Speed limits: 17mm 25m/s(42000-22~23m/s), 42mm 15m/s
static void FrictionOutput()
{
    // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
    switch (shoot_cmd_recv.bullet_speed)
    {
    case SMALL_AMU_15:
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        break;
    case SMALL_AMU_22:
        DJIMotorSetRef(friction_l, 42000);
        DJIMotorSetRef(friction_r, 42000);
        break;
    default:
        DJIMotorSetRef(friction_l, 42000);
        DJIMotorSetRef(friction_r, 42000);
        break;
    }
}

/* 发射输出逻辑, 仅在卡弹状态机正常/嫌疑态时执行
 * 仿中科大Class_Booster::Output(): 按发射机构状态shoot_cmd_recv.shoot_state分四种模式 */
static void ShootOutput()
{
    // 失能: 上电默认/急停, 不真正停机, 而是给零参考(仿中科大DISABLE)
    //   拨盘走电流环参考0(零电流), 两摩擦轮走速度环参考0
    if (shoot_cmd_recv.shoot_state == BOOSTER_DISABLE)
    {
        DJIMotorOuterLoop(loader, CURRENT_LOOP);
        DJIMotorSetRef(loader, 0);
        DJIMotorOuterLoop(friction_l, SPEED_LOOP);
        DJIMotorOuterLoop(friction_r, SPEED_LOOP);
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        return; // 失能态不再设定其它参考值
    }

    // 飞轮常转(非失能态), 飞轮一直在转
    FrictionOutput();

    // 如果上一次触发单发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 不应期内忽略后续状态切换(例如触发单发后立即回停火),保证拨盘走完一发弹丸的角度
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    // 拨盘控制: 按发射机构状态进行参考值设定
    switch (shoot_cmd_recv.shoot_state)
    {
    // 停火: 拨盘停在原地(速度环参考0,停得最快), 飞轮保持转动
    case BOOSTER_CEASEFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;

    // 单发: 角度环走一发弹丸的角度,触发后进入不响应输入的休眠期(150ms内忽略重复触发,保证只发一颗)
    // 单发后回到停火由cmd负责(鼠标上升沿发SPOT一帧,之后即CEASEFIRE)
    case BOOSTER_SPOT:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                       // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle - ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                        // 记录触发指令的时间
        dead_time = 150;                                                              // 完成1发弹丸发射的时间
        break;

    // 连发: 对速度闭环,根据枪口剩余热量(barrel heat余量)动态选择拨弹射频
    // 仿中科大开源步兵Booster_Control_Type_AUTO热量控制逻辑:
    //   余量充足时全速发射;余量进入缓冲区后在目标射频与可持续射频(冷却速率)之间线性过渡;余量过低时停火
    case BOOSTER_AUTO:
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
    default:
        break;
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
        // 安全: 调拨盘时摩擦轮给零速参考(不真正停机,退出调参后能正常恢复)
        DJIMotorOuterLoop(friction_l, SPEED_LOOP);
        DJIMotorOuterLoop(friction_r, SPEED_LOOP);
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);

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

    // 发射机构状态由cmd直接给定(单一状态机, shoot_cmd_recv.shoot_state):
    //   上电默认失能,第一次触发单发/连发后激活,之后在停火(飞轮常转)与发射态间切换,只有急停才回失能
    // 失能时复位卡弹状态机,保证 ShootOutput() 真正停机, FSM 不在失能期间劫持拨盘(安全保护)
    if (shoot_cmd_recv.shoot_state == BOOSTER_DISABLE)
        ShootJamSetStatus(JAM_NORMAL);

    // 卡弹处理状态机: 仿中科大Class_Booster
    // 正常/嫌疑态执行正常发射输出; 确认/处理态由FSM接管拨盘回拨, 摩擦轮保持上次转速
    ShootJamFSM();
    if (jam_status == JAM_NORMAL || jam_status == JAM_SUSPECT)
        ShootOutput();

    dbg_jam_status = (uint8_t)jam_status;
    dbg_booster_type = (uint8_t)shoot_cmd_recv.shoot_state;
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