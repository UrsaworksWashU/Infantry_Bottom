#include "press_hold_fsm.h"

#define PRESS_HOLD_DEFAULT_THRESHOLD 50u // 默认长按阈值(周期数);200Hz下=250ms

/* 切换状态,并清零该状态的计时(对应中科大的 Set_Status) */
static void SetStatus(PressHoldFSM_t *fsm, PressHold_Status_e next)
{
    fsm->status = next;
    fsm->count_time = 0;
}

void PressHoldFSM_Init(PressHoldFSM_t *fsm, uint16_t hold_threshold)
{
    fsm->status = FSM_PRESS_HOLD_STOP;
    fsm->count_time = 0;
    fsm->hold_threshold = (hold_threshold != 0) ? hold_threshold : PRESS_HOLD_DEFAULT_THRESHOLD;
    fsm->last_press = 0;
}

PressHold_Status_e PressHoldFSM_Update(PressHoldFSM_t *fsm, uint8_t pressed)
{
    fsm->count_time++;

    // 由原始按下信号(0/1)结合上一周期状态得到边沿事件,
    // 等价于中科大的 TRIG_FREE_PRESSED(上升沿) / TRIG_PRESSED_FREE(下降沿)
    uint8_t rising = (pressed && !fsm->last_press);  // 松->按 上升沿
    uint8_t falling = (!pressed && fsm->last_press); // 按->松 下降沿

    switch (fsm->status)
    {
    case FSM_PRESS_HOLD_STOP: // 停机状态
        if (rising)
            SetStatus(fsm, FSM_PRESS_HOLD_PRESSED); // 停机 -> 按下
        break;

    case FSM_PRESS_HOLD_FREE: // 松开状态
        if (rising)
            SetStatus(fsm, FSM_PRESS_HOLD_PRESSED); // 松开 -> 按下
        break;

    case FSM_PRESS_HOLD_PRESSED:                    // 按下状态:在此判定单点 or 长按
        if (falling)                                // 阈值前就松开 -> 单点
            SetStatus(fsm, FSM_PRESS_HOLD_CLICK);
        else if (fsm->count_time >= fsm->hold_threshold) // 持续按到阈值 -> 长按
            SetStatus(fsm, FSM_PRESS_HOLD_HOLD);
        break;

    case FSM_PRESS_HOLD_CLICK: // 单点状态:瞬时,下一周期立即回到松开(保证单点事件只触发一次)
        SetStatus(fsm, FSM_PRESS_HOLD_FREE);
        break;

    case FSM_PRESS_HOLD_HOLD: // 长按状态
        if (!pressed)
            SetStatus(fsm, FSM_PRESS_HOLD_FREE); // 完全松开 -> 松开
        break;

    default:
        break;
    }

    fsm->last_press = pressed;
    return fsm->status;
}
