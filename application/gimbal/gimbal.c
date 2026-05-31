#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include <math.h>

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;
// static float pitch_neg;  // IMU pitch取反，用于方向修正
// static float gyro0_neg;  // IMU Gyro[0]取反，用于方向修正

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// Ozone Timeline调试变量
volatile float dbg_pitch_speed_ref;
volatile float dbg_pitch_speed_measure;
volatile float dbg_pitch_angle_ref;
volatile float dbg_pitch_angle_measure;

// pitch重力补偿前馈，电流单位（同speed PID MaxOut），Ozone实时调参，调好后写入robot_def.h
static float   pitch_gravity_ff      = 0.0f; // 由motor_task 1kHz读取，必须是static全局
volatile float pitch_gravity_ff_coef = PITCH_GRAVITY_FF_COEF; // Ozone可实时改，正值=向上补偿，符号不对则取负

// 速度环调参：Ozone把 tuning 设为1进入，0恢复正常
// 调哪个电机就取消对应块的注释，另一个保持注释
volatile uint8_t tuning         = 0;
volatile float   yaw_tune_amp   = 600.0f; // deg/s，Ozone可实时改
volatile float   yaw_tune_freq  = 0.5f;   // Hz，Ozone可实时改
volatile float   pitch_tune_amp = 100.0f; // deg/s，pitch行程小幅值保守
volatile float   pitch_tune_freq= 0.5f;   // Hz，Ozone可实时改

void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, 
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 150,  
                .Ki = 200, 
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // Gyro[2]是rad/s，speed PID需要deg/s，所以指向转换后的yaw_gyro_dps
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro_dps[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 12,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 120,
                .Ki = 20,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            // Gyro[0]是rad/s，speed PID需要deg/s，所以指向转换后的pitch_gyro_dps
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro_dps[0],
            .current_feedforward_ptr  = &pitch_gravity_ff,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag   = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // TODO: add code for reversed imu direction for future
    // pitch_neg  = -gimbal_IMU_data->Pitch;
    // gyro0_neg  = -gimbal_IMU_data->Gyro[0];

    if (tuning)
    {
        float t = DWT_GetTimeline_s();

        // // === YAW 速度环调参（调pitch时注释此块）===
        // DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
        // DJIMotorEnable(yaw_motor);
        // DJIMotorStop(pitch_motor);
        // // 方波阶跃
        // // float yaw_half = 0.5f / yaw_tune_freq;
        // // float yaw_ref = (fmodf(t, 2.0f * yaw_half) < yaw_half) ? yaw_tune_amp : -yaw_tune_amp;
        // // 正弦波（备用）
        // float yaw_ref = yaw_tune_amp * sinf(2.0f * 3.14159265f * yaw_tune_freq * t);
        // DJIMotorSetRef(yaw_motor, yaw_ref);

        // === PITCH 速度环调参（调yaw时注释此块）===
        DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
        DJIMotorEnable(pitch_motor);
        DJIMotorStop(yaw_motor);
        // 方波阶跃
        float pitch_half = 0.5f / pitch_tune_freq;
        // float pitch_ref = (fmodf(t, 2.0f * pitch_half) < pitch_half) ? pitch_tune_amp : -pitch_tune_amp;
        // 正弦波（备用）
        float pitch_ref = pitch_tune_amp * sinf(2.0f * 3.14159265f * pitch_tune_freq * t);
        // 角度限位：到达边界时只允许反向运动，同时清空积分防止windup反弹
        float pitch_now = gimbal_IMU_data->Pitch;
        if ((pitch_now >= PITCH_MAX_ANGLE && pitch_ref > 0) ||
            (pitch_now <= PITCH_MIN_ANGLE && pitch_ref < 0)) {
            pitch_ref = 0;
            pitch_motor->motor_controller.speed_PID.ITerm      = 0;
            pitch_motor->motor_controller.speed_PID.Iout       = 0;
            pitch_motor->motor_controller.speed_PID.Last_ITerm = 0;
        }
        DJIMotorSetRef(pitch_motor, pitch_ref);
    }
    else
    {
        // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
        // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
        DJIMotorOuterLoop(yaw_motor, ANGLE_LOOP);
        DJIMotorOuterLoop(pitch_motor, ANGLE_LOOP);
        switch (gimbal_cmd_recv.gimbal_mode)
        {
        // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            DJIMotorStop(pitch_motor);
            break;
        // 视觉自瞄：角度+速度双环，IMU反馈，pitch软件限位
        case GIMBAL_ANGLE_MODE:
            DJIMotorEnable(yaw_motor);
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
            { // pitch软件限位
                float pitch_ref = gimbal_cmd_recv.pitch;
                if (pitch_ref > PITCH_MAX_ANGLE) pitch_ref = PITCH_MAX_ANGLE;
                if (pitch_ref < PITCH_MIN_ANGLE) pitch_ref = PITCH_MIN_ANGLE;
                DJIMotorSetRef(pitch_motor, pitch_ref);
            }
            break;
        // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
        case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
            DJIMotorEnable(yaw_motor);
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED);
            DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
            { // pitch软件限位
                float pitch_ref = gimbal_cmd_recv.pitch;
                if (pitch_ref > PITCH_MAX_ANGLE) pitch_ref = PITCH_MAX_ANGLE;
                if (pitch_ref < PITCH_MIN_ANGLE) pitch_ref = PITCH_MIN_ANGLE;
                DJIMotorSetRef(pitch_motor, pitch_ref);
            }
            break;
        default:
            break;
        }
    }

    // pitch重力补偿前馈：注入到speed PID输出之后（电流单位），motor_task 1kHz读取
    pitch_gravity_ff = pitch_gravity_ff_coef * cosf(gimbal_IMU_data->Pitch * (3.14159265f / 180.0f));

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    dbg_pitch_speed_ref     = pitch_motor->motor_controller.speed_PID.Ref;
    dbg_pitch_speed_measure = pitch_motor->motor_controller.speed_PID.Measure;
    dbg_pitch_angle_ref     = pitch_motor->motor_controller.angle_PID.Ref;
    dbg_pitch_angle_measure = pitch_motor->motor_controller.angle_PID.Measure;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}