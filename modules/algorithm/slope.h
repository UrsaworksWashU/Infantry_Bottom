#ifndef SLOPE_H
#define SLOPE_H

#include <stdint.h>

/*
 * 斜坡(速度规划)函数
 * ----------------------------------------------------------------------------
 * 仿中科大(USTC)开源框架的 Class_Slope, 移植为本项目的C风格。
 *
 * 作用: 让输出从当前值按"每周期限定步长"平滑逼近目标值, 实现可控的加/减速,
 *       例如键盘WASD"按得越久速度越快", 松开后平滑刹停。
 *
 * - increase_value: 每周期幅值"增长"上限(远离当前向目标加速时用)
 * - decrease_value: 每周期幅值"降低"上限(向目标减速/刹车时用)
 *   两者分开, 便于"缓加速、急刹车"。
 *
 * 用法: 固定周期内每次调用一次 SlopeCalc(slope, target), 返回本周期输出。
 */

typedef enum
{
    SLOPE_FIRST_PLANNING = 0, // 以规划值(上一次输出)为基准(默认)
    SLOPE_FIRST_REAL,         // 以真实反馈值为基准(需先调用SlopeSetReal提供反馈)
} Slope_First_e;

typedef struct
{
    float increase_value; // 每周期最大增长幅度(加速)
    float decrease_value; // 每周期最大降低幅度(减速)
    Slope_First_e slope_first;

    float target;       // 目标值
    float now_real;     // 当前真实值(SLOPE_FIRST_REAL时使用)
    float now_planning; // 上一周期规划输出
    float out;          // 本周期输出
} SlopeInstance;

/**
 * @brief 初始化斜坡规划器
 *
 * @param slope          实例
 * @param increase_value 每周期加速步长(>0)
 * @param decrease_value 每周期减速步长(>0)
 * @param slope_first    基准选择, 一般用 SLOPE_FIRST_PLANNING
 */
void SlopeInit(SlopeInstance *slope, float increase_value, float decrease_value, Slope_First_e slope_first);

/**
 * @brief 提供真实反馈值(仅 SLOPE_FIRST_REAL 模式需要)
 */
void SlopeSetReal(SlopeInstance *slope, float now_real);

/**
 * @brief 计算一个周期的斜坡输出
 *
 * @param slope  实例
 * @param target 目标值
 * @return float 本周期规划输出(= slope->out)
 */
float SlopeCalc(SlopeInstance *slope, float target);

#endif // SLOPE_H
