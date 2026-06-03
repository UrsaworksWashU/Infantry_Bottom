/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    uint16_t flag_register;
    DaemonReload(vision_daemon_instance); // 喂狗
    get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
    recv_data.fire_mode    = (Fire_Mode_e)(flag_register & 0x03);
    recv_data.target_state = (Target_State_e)((flag_register >> 2) & 0x03);
    recv_data.target_type  = (Target_Type_e)((flag_register >> 8) & 0xFF);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
#include "ins_task.h"
#include "string.h"

static uint8_t *vis_recv_buff;

/* CRC-16/IBM-SDLC (poly=0x8408, init=0xFFFF) — must match Vision26 tools/crc.cpp */
static const uint16_t SP_CRC16_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

static uint16_t sp_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xffff;
    while (len--)
        crc = (crc >> 8) ^ SP_CRC16_TABLE[(crc ^ *data++) & 0xff];
    return crc;
}

/*
 * SP protocol wire layout (Vision26 io/gimbal/gimbal.hpp, __attribute__((packed))):
 *
 * GimbalToVision (STM32 → Jetson), 43 bytes:
 *   [0]      'S'
 *   [1]      'P'
 *   [2]      mode  (0=idle, 1=auto_aim, 2=small_buff, 3=big_buff)
 *   [3–6]    q[0] w  (float LE)
 *   [7–10]   q[1] x
 *   [11–14]  q[2] y
 *   [15–18]  q[3] z
 *   [19–22]  yaw       (float, deg)
 *   [23–26]  yaw_vel   (float, rad/s)
 *   [27–30]  pitch     (float, deg)
 *   [31–34]  pitch_vel (float, rad/s)
 *   [35–38]  bullet_speed (float, m/s)
 *   [39–40]  bullet_count (uint16 LE)
 *   [41–42]  crc16 (LE)
 *
 * VisionToGimbal (Jetson → STM32), 29 bytes:
 *   [0]      'S'
 *   [1]      'P'
 *   [2]      mode  (0=no ctrl, 1=ctrl no fire, 2=ctrl+fire)
 *   [3–6]    yaw       (float, deg)
 *   [7–10]   yaw_vel
 *   [11–14]  yaw_acc
 *   [15–18]  pitch     (float, deg)
 *   [19–22]  pitch_vel
 *   [23–26]  pitch_acc
 *   [27–28]  crc16 (LE)
 *
 * Note: floats sit at odd byte offsets in the packed wire format. On Cortex-M4 the
 * FPU (VLDR/VSTR) requires 4-byte alignment and will HardFault on unaligned access.
 * All float I/O therefore goes through memcpy into aligned local variables.
 */
#define GIMBAL_TO_VISION_SIZE 43u
#define VISION_TO_GIMBAL_SIZE 29u

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len < VISION_TO_GIMBAL_SIZE) return;

    const uint8_t *p = vis_recv_buff;
    if (p[0] != 'S' || p[1] != 'P') return;

    uint16_t crc = (uint16_t)p[VISION_TO_GIMBAL_SIZE - 2] |
                   ((uint16_t)p[VISION_TO_GIMBAL_SIZE - 1] << 8);
    if (sp_crc16(p, VISION_TO_GIMBAL_SIZE - 2) != crc) return;

    DaemonReload(vision_daemon_instance);

    memcpy(&recv_data.yaw,   p + 3,  4);
    memcpy(&recv_data.pitch, p + 15, 4);

    switch (p[2]) {
        case 2:
            recv_data.fire_mode    = AUTO_FIRE;
            recv_data.target_state = READY_TO_FIRE;
            break;
        case 1:
            recv_data.fire_mode    = AUTO_AIM;
            recv_data.target_state = TARGET_CONVERGING;
            break;
        default:
            recv_data.fire_mode    = NO_FIRE;
            recv_data.target_state = NO_TARGET;
            break;
    }
}

/* 视觉通信初始化
 * Jetson端需建立udev规则将/dev/ttyACM0映射为/dev/gimbal:
 *   SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="00ca", SYMLINK+="gimbal"
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle);
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback,
        .owner_id     = NULL,
        .reload_count = 5,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint8_t buf[GIMBAL_TO_VISION_SIZE];
    float q[4];
    float zero = 0.0f;
    float bullet_spd;
    uint16_t count = 0;
    uint16_t crc;
    uint8_t mode;

    switch (send_data.work_mode) {
        case VISION_MODE_SMALL_BUFF: mode = 2; break;
        case VISION_MODE_BIG_BUFF:   mode = 3; break;
        case VISION_MODE_AIM:        mode = 1; break;
        default:                     mode = 0; break;
    }

    /* 直接发送INS原始四元数(wxyz),避免欧拉角往返转换的精度/约定损失 */
    const float *ins_q = INS_GetQuaternion();
    q[0] = ins_q[0];
    q[1] = ins_q[1];
    q[2] = ins_q[2];
    q[3] = ins_q[3];
    bullet_spd = (float)send_data.bullet_speed;

    buf[0] = 'S';
    buf[1] = 'P';
    buf[2] = mode;
    memcpy(buf + 3,  &q[0],            4);
    memcpy(buf + 7,  &q[1],            4);
    memcpy(buf + 11, &q[2],            4);
    memcpy(buf + 15, &q[3],            4);
    memcpy(buf + 19, &send_data.yaw,   4);
    memcpy(buf + 23, &zero,            4); /* yaw_vel */
    memcpy(buf + 27, &send_data.pitch, 4);
    memcpy(buf + 31, &zero,            4); /* pitch_vel */
    memcpy(buf + 35, &bullet_spd,      4);
    memcpy(buf + 39, &count,           2); /* bullet_count */

    crc = sp_crc16(buf, GIMBAL_TO_VISION_SIZE - 2);
    buf[41] = (uint8_t)(crc & 0xff);
    buf[42] = (uint8_t)(crc >> 8);

    USBTransmit(buf, GIMBAL_TO_VISION_SIZE);
}

#endif // VISION_USE_VCP
