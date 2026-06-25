#ifndef PRESS_HOLD_FSM_H
#define PRESS_HOLD_FSM_H

#include <stdint.h>

/*
 * 按键"单点 / 长按"判断状态机
 * ----------------------------------------------------------------------------
 * 仿照中科大(USTC)开源框架的 Class_FSM_Press_Hold 思路,移植为本项目的C风格。
 *
 * 状态流转:
 *   STOP/FREE --(按下上升沿)--> PRESSED
 *   PRESSED   --(阈值前松开)--> CLICK --> FREE      (产生一次"单点"事件)
 *   PRESSED   --(按住超阈值)--> HOLD                (进入"长按")
 *   HOLD      --(完全松开)----> FREE
 *
 * 用法:每个控制周期调用一次 PressHoldFSM_Update(),传入按键当前是否按下(0/1)。
 *   - CLICK 状态只维持一个周期 => 用 PressHoldFSM_IsClick() 判定"单点"一次性事件
 *   - HOLD  状态在长按期间持续为真 => 用 PressHoldFSM_IsHold() 判定"长按"持续动作
 *
 * 计时单位:count_time 以"调用次数"计,需在固定频率下调用。
 *   例:在 RobotCMDTask(200Hz, 每tick=5ms)中调用,hold_threshold=50 即 250ms。
 */

typedef enum
{
    FSM_PRESS_HOLD_STOP = 0, // 停机/初始状态
    FSM_PRESS_HOLD_FREE,     // 松开状态
    FSM_PRESS_HOLD_PRESSED,  // 按下状态(尚未判定单点/长按)
    FSM_PRESS_HOLD_CLICK,    // 单点状态(瞬时,仅维持一个周期)
    FSM_PRESS_HOLD_HOLD,     // 长按状态
} PressHold_Status_e;

typedef struct
{
    PressHold_Status_e status; // 当前状态
    uint32_t count_time;       // 当前状态已停留的周期数(进入新状态时清零)
    uint16_t hold_threshold;   // 进入长按的阈值(周期数)
    uint8_t last_press;        // 上一周期的按下信号,用于检测边沿
} PressHoldFSM_t;

/**
 * @brief 初始化按键单点/长按状态机
 *
 * @param fsm            状态机实例
 * @param hold_threshold 长按判定阈值(调用周期数);传0则用内部默认值
 */
void PressHoldFSM_Init(PressHoldFSM_t *fsm, uint16_t hold_threshold);

/**
 * @brief 状态机更新,每个控制周期调用一次
 *
 * @param fsm     状态机实例
 * @param pressed 当前按键是否按下(0/1),如 rc_data[TEMP].mouse.press_l
 * @return        更新后的当前状态
 */
PressHold_Status_e PressHoldFSM_Update(PressHoldFSM_t *fsm, uint8_t pressed);

/* 便捷查询:是否发生一次"单点"(仅在 CLICK 周期为真) */
static inline uint8_t PressHoldFSM_IsClick(const PressHoldFSM_t *fsm)
{
    return fsm->status == FSM_PRESS_HOLD_CLICK;
}

/* 便捷查询:是否处于"长按" */
static inline uint8_t PressHoldFSM_IsHold(const PressHoldFSM_t *fsm)
{
    return fsm->status == FSM_PRESS_HOLD_HOLD;
}

#endif // PRESS_HOLD_FSM_H
