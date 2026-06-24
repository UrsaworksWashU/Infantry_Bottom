#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 18u // UART legacy size (seasky protocol)
#define VISION_SEND_SIZE 36u // UART legacy size (seasky protocol)

// VISION_USE_VCP: rm_serial_driver compatible protocol
// C-board -> Jetson: ReceivePacket (0x5A header, 34 bytes)
// Jetson -> C-board: SendPacket    (0xA5 header, 12 bytes)
#define GIMBAL_TO_JETSON_SIZE 34u
#define JETSON_TO_GIMBAL_SIZE 12u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch;
	float yaw;
	// 轨迹规划器前馈量（单位与yaw/pitch一致：rad / rad/s / rad/s²）
	float yaw_vel;
	float yaw_acc;
	float pitch_vel;
	float pitch_acc;
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_22 = 22,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;     // 弹速档位枚举(UART协议flag位使用)
	float bullet_speed_real;         // 裁判系统实测弹速(m/s),用于上位机弹道解算
	Enemy_Color_e self_color;        // 本机颜色(裁判系统读取),回传上位机以确定要识别的敌方颜色

	float yaw;
	float pitch;
	float roll;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置回传给上位机的实测弹速(来自裁判系统0x0207实时射击数据)
 *
 * @param speed 弹速,单位m/s
 */
void VisionSetBulletSpeed(float speed);

/**
 * @brief 设置回传给上位机的本机颜色(红/蓝),上位机据此识别敌方装甲板
 *
 * @param color COLOR_NONE/COLOR_BLUE/COLOR_RED,来自裁判系统机器人ID
 */
void VisionSetSelfColor(Enemy_Color_e color);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

#endif // !MASTER_PROCESS_H