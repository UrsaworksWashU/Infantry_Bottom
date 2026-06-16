#ifndef SHOOT_H
#define SHOOT_H

// 连发热量控制参数 (实测后修改)
#define HEAT_PER_BULLET          10.0f  // 17mm 弹丸每发热量
#define HEAT_SLOWDOWN_THRESHOLD  40.0f  // 余量高于此值全速发射
#define HEAT_CEASEFIRE_THRESHOLD 10.0f  // 余量低于此值停火

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