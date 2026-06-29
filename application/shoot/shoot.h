#ifndef SHOOT_H
#define SHOOT_H

// 连发热量控制参数 (实测后修改)
#define HEAT_PER_BULLET          10.0f  // 17mm 弹丸每发热量
#define HEAT_SLOWDOWN_THRESHOLD  50.0f  // 余量高于此值全速发射
#define HEAT_CEASEFIRE_THRESHOLD 30.0f  // 余量低于此值停火

// === 发射自检热量计算参数, 仿中科大"无裁判系统热量检测算法" (实测后修改) ===
// 原理: 一发弹丸挤过摩擦轮时, 速度环为维持转速会瞬间拉高扭矩电流, 以此突变检测每一发实际出弹,
//       本地即时累加热量(Qnow), 取代裁判系统约100ms的延迟反馈; 再用裁判系统读数作安全下限兜底.
#define FRICTION_FIRE_CURRENT_THRESHOLD 3000     // 出弹瞬间摩擦轮扭矩电流(real_current绝对值)阈值, 需高于空载稳态电流; Ozone看dbg_friction_fire_current标定(约对应中科大1.4A参考)
#define FRICTION_FIRE_CONFIRM_MS        10.0f    // 扭矩电流持续高于阈值此时长才确认一发(去抖, 滤除噪声毛刺; 中科大参考20ms, 本任务5ms周期下取10ms)
#define FRICTION_READY_SPEED_APS        30000.0f // 摩擦轮转速(speed_aps绝对值,deg/s)高于此才视为飞轮就绪可检测出弹, 约目标转速的0.7倍
#define HEAT_LIMIT_FALLBACK             90.0f   // 裁判系统离线(热量上限读为0)时兜底的枪口热量上限
#define BARREL_COOLING_FALLBACK         40.0f    // 裁判系统离线时兜底的每秒冷却值(J/s)

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