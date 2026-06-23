// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
    shoot_cmd_send.shoot_state = BOOSTER_DISABLE; // 上电默认失能,第一次触发单发/连发后才激活
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    float angle = gimbal_fetch_data.yaw_motor_single_round_angle;
    float offset = angle - YAW_ALIGN_ANGLE;
    if (offset > 180.0f)  offset -= 360.0f;
    if (offset < -180.0f) offset += 360.0f;
    chassis_cmd_send.offset_angle = offset;
}

/**
 * @brief Control logics and coefficients when using remote controller
 *
 */
static void RemoteControlSet()
{
    // Control logics for chassis and gimbal when using remote controller
    // When right switch is down, gimbal in angle mode and chassis in rotation mode
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CLOCKWISE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ANGLE_MODE;
    }
    // When right switch is mid, gimbal in angle mode and chassis follow gimbal yaw
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ANGLE_MODE;
    }

    // Gimbal control coefficients, right stick horizontal for yaw, vertical for pitch
    // horizontal controller stick negative left, positive right
    gimbal_cmd_send.pitch +=  0.001f * (float)rc_data[TEMP].rc.rocker_r1; // r1 for horizontal direction
    gimbal_cmd_send.yaw   += -0.005f * (float)rc_data[TEMP].rc.rocker_r_; // r for vertical direction
    gimbal_cmd_send.yaw_speed_ff   = 0.0f; // 手动模式无前馈
    gimbal_cmd_send.pitch_speed_ff = 0.0f;
    // Put it here rather than gimbal.c so pitch can have continuous control when hitting the limit, 
    // it doesn't need to counteract the exceeded values
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

    // Chassis control coefficients, left stick vertical for forward/backward, horizontal for left/right
    // Negative sign to align controller output with RHR(right hand rule: x forward, y left, wz counterclockwise positive)
    chassis_cmd_send.vx =  10.0f * (float)rc_data[TEMP].rc.rocker_l1; // l1 for vertical direction
    chassis_cmd_send.vy = -10.0f * (float)rc_data[TEMP].rc.rocker_l_; // l_ for horizontal direction

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    else
        ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    // 发射机构状态控制(拨轮向上打为负): 回中失能, 上拨过半停火(飞轮常转), 上拨到底连发
    if (rc_data[TEMP].rc.dial < -500) // 上拨到底: 连发
        shoot_cmd_send.shoot_state = BOOSTER_AUTO;
    else if (rc_data[TEMP].rc.dial < -100) // 上拨过半: 飞轮常转,不发弹
        shoot_cmd_send.shoot_state = BOOSTER_CEASEFIRE;
    else // 回中: 失能,飞轮停
        shoot_cmd_send.shoot_state = BOOSTER_DISABLE;
}

/**
 * @brief 控制输入为视觉上位机时的模式和控制量设置
 *
 */
  static void VisionControlSet()
  {
      gimbal_cmd_send.gimbal_mode = GIMBAL_ANGLE_MODE;
      chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; 
      if (vision_recv_data->target_state != NO_TARGET)
      {
          float target_yaw = vision_recv_data->yaw * 57.2958f;   // absolute, (-180,180] deg
          float err = target_yaw - gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
          while (err >  180.0f) err -= 360.0f;                    // wrap to shortest path
          while (err < -180.0f) err += 360.0f;
          gimbal_cmd_send.yaw   = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle + err;
          gimbal_cmd_send.pitch = -vision_recv_data->pitch * 57.2958f;
          // 视觉轨迹规划器的速度前馈，rad/s -> deg/s，直接喂给云台速度环
          // pitch与角度一致取负，保持方向定义统一
          gimbal_cmd_send.yaw_speed_ff   =  vision_recv_data->yaw_vel   * 57.2958f;
          gimbal_cmd_send.pitch_speed_ff = -vision_recv_data->pitch_vel * 57.2958f;
      }
      else
      {
          gimbal_cmd_send.yaw_speed_ff   = 0.0f; // 无目标，关闭前馈
          gimbal_cmd_send.pitch_speed_ff = 0.0f;
      }
        // 发射机构状态控制(拨轮向上打为负): 上拨到底且视觉判定可开火->连发, 上拨过半->飞轮常转待发, 回中->失能
        if (rc_data[TEMP].rc.dial < -500 && vision_recv_data->target_state == READY_TO_FIRE)
            shoot_cmd_send.shoot_state = BOOSTER_AUTO;
        else if (rc_data[TEMP].rc.dial < -100)
            shoot_cmd_send.shoot_state = BOOSTER_CEASEFIRE;
        else
            shoot_cmd_send.shoot_state = BOOSTER_DISABLE;
        }

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_ANGLE_MODE;
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

    chassis_cmd_send.vx = 1000.0f * (float)rc_data[TEMP].key[KEY_PRESS].w - 1000.0f * (float)rc_data[TEMP].key[KEY_PRESS].s; // 系数待测
    chassis_cmd_send.vy = 1000.0f * (float)rc_data[TEMP].key[KEY_PRESS].d - 1000.0f * (float)rc_data[TEMP].key[KEY_PRESS].a;

    if (rc_data[TEMP].key[KEY_PRESS].q) 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_COUNTERCLOCKWISE;
    }
    else if (rc_data[TEMP].key[KEY_PRESS].e)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_CLOCKWISE;
    }
    else
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }

    gimbal_cmd_send.yaw   += -(float)rc_data[TEMP].mouse.x * 0.005f;
    gimbal_cmd_send.pitch +=  (float)rc_data[TEMP].mouse.y * 0.001f;
    gimbal_cmd_send.yaw_speed_ff   = 0.0f; // 键鼠模式无前馈
    gimbal_cmd_send.pitch_speed_ff = 0.0f;
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE) gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE) gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    // {
    // case 0:
    //     shoot_cmd_send.bullet_speed = 15;
    //     break;
    // case 1:
    //     break;
    // default:
    //     shoot_cmd_send.bullet_speed = 30;
    //     break;
    // }
    // 鼠标左键发射控制(直接驱动发射机构状态机,仅保留长按连发,无单发):
    //   按住超过HOLD_TO_BURST_TIME_MS->BOOSTER_AUTO连发
    //   未达长按阈值/已松开->BOOSTER_CEASEFIRE停火(飞轮保持转,首次连发前仍保持失能)
    static uint8_t mouse_l_last      = 0; // 上一帧左键状态,用于检测上升沿(记录按下时刻)
    static float   mouse_l_press_ms  = 0; // 左键按下的时刻
    uint8_t mouse_l_now = rc_data[TEMP].mouse.press_l;
    float   now_ms      = DWT_GetTimeline_ms();

    if (mouse_l_now && !mouse_l_last) // 刚按下:记录按下时刻,用于长按判定
        mouse_l_press_ms = now_ms;

    if (mouse_l_now && (now_ms - mouse_l_press_ms >= HOLD_TO_BURST_TIME_MS)) // 长按:连发
    {
        shoot_cmd_send.shoot_state = BOOSTER_AUTO;
    }
    else // 未达长按阈值,或已松开:已激活过才进停火,否则保持上电失能态
    {
        if (shoot_cmd_send.shoot_state != BOOSTER_DISABLE)
            shoot_cmd_send.shoot_state = BOOSTER_CEASEFIRE;
    }
    mouse_l_last = mouse_l_now;

    if (rc_data[TEMP].key[KEY_PRESS].f) // F键:手动失能发射机构(停飞轮),需重新点击鼠标才再次激活
    {
        shoot_cmd_send.shoot_state = BOOSTER_DISABLE;
    }

    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    // {
    // case 0:
    //     shoot_cmd_send.lid_mode = LID_OPEN;
    //     break;
    // default:
    //     shoot_cmd_send.lid_mode = LID_CLOSE;
    //     break;
    // }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    // {
    // case 0:
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     break;
    // default:
    //     shoot_cmd_send.friction_mode = FRICTION_ON;
    //     break;
    // }
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    // {
    // case 0:
    //     chassis_cmd_send.chassis_speed_buff = 40;
    //     break;
    // case 1:
    //     chassis_cmd_send.chassis_speed_buff = 60;
    //     break;
    // case 2:
    //     chassis_cmd_send.chassis_speed_buff = 80;
    //     break;
    // default:
    //     chassis_cmd_send.chassis_speed_buff = 100;
    //     break;
    // }
    // switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    // {
    // case 1:

    //     break;

    // default:

    //     break;
    // }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_state = BOOSTER_DISABLE; // 急停:发射机构失能(含飞轮)
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行(发射机构状态由各输入处理函数决定,不在此强制使能)
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[中],视觉模式
        VisionControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color, VISION_MODE_AIM, chassis_fetch_data.bullet_speed);
    VisionSetFlag(COLOR_RED, VISION_MODE_AIM, SMALL_AMU_15); // TODO: get color/speed from referee
    // 发送当前云台姿态给上位机用于弹道解算 不再使用 INS里会发送 因为那个是1000Hz task 频率更高
    // VisionSetAltitude(gimbal_fetch_data.gimbal_imu_data.Yaw/57.2958f,
    //                   gimbal_fetch_data.gimbal_imu_data.Pitch/57.2958f,
    //                   gimbal_fetch_data.gimbal_imu_data.Roll/57.2958f);
    VisionSend();

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
