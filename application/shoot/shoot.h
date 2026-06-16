#ifndef SHOOT_H
#define SHOOT_H

// 连发热量控制参数 (实测后修改)
#define HEAT_PER_BULLET          10.0f  // 17mm 弹丸每发热量
#define HEAT_SLOWDOWN_THRESHOLD  40.0f  // 余量高于此值全速发射
#define HEAT_CEASEFIRE_THRESHOLD 20.0f  // 余量低于此值停火

// 卡弹处理(anti-jamming)参数, 仿中科大 (实测后修改)
#define JAM_CURRENT_THRESHOLD  2000     // 拨盘堵转电流阈值(real_current绝对值), 超过视为卡弹嫌疑 -10A~0~10A -> -10000~0~10000 
#define JAM_SUSPECT_TIME_MS    300.0f   // 持续大电流达到此时长判定为卡弹(ms)
#define JAM_SOLVING_TIME_MS    200.0f   // 回拨处理持续时间(ms)
#define JAM_BACK_ANGLE         (ONE_BULLET_DELTA_ANGLE)  // 回拨角度, 与单发同单位, 默认退一发

/**
 * @brief 发射初始化,会被RobotInit()调用
 *
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

#endif // SHOOT_H