#include "slope.h"
#include <math.h>

void SlopeInit(SlopeInstance *slope, float increase_value, float decrease_value, Slope_First_e slope_first)
{
    slope->increase_value = increase_value;
    slope->decrease_value = decrease_value;
    slope->slope_first = slope_first;
    slope->target = 0.0f;
    slope->now_real = 0.0f;
    slope->now_planning = 0.0f;
    slope->out = 0.0f;
}

void SlopeSetReal(SlopeInstance *slope, float now_real)
{
    slope->now_real = now_real;
}

float SlopeCalc(SlopeInstance *slope, float target)
{
    slope->target = target;

    // 以真实值为基准: 若真实值已越过规划值朝目标方向前进, 直接对齐到真实值, 避免规划滞后
    if (slope->slope_first == SLOPE_FIRST_REAL)
    {
        if ((slope->target >= slope->now_real && slope->now_real >= slope->now_planning) ||
            (slope->target <= slope->now_real && slope->now_real <= slope->now_planning))
        {
            slope->out = slope->now_real;
        }
    }

    if (slope->now_planning > 0.0f)
    {
        if (slope->target > slope->now_planning) // 正值加速(幅值增大)
        {
            if (fabsf(slope->now_planning - slope->target) > slope->increase_value)
                slope->out += slope->increase_value;
            else
                slope->out = slope->target;
        }
        else if (slope->target < slope->now_planning) // 正值减速(幅值减小)
        {
            if (fabsf(slope->now_planning - slope->target) > slope->decrease_value)
                slope->out -= slope->decrease_value;
            else
                slope->out = slope->target;
        }
    }
    else if (slope->now_planning < 0.0f)
    {
        if (slope->target < slope->now_planning) // 负值加速(幅值增大)
        {
            if (fabsf(slope->now_planning - slope->target) > slope->increase_value)
                slope->out -= slope->increase_value;
            else
                slope->out = slope->target;
        }
        else if (slope->target > slope->now_planning) // 负值减速(幅值减小, 朝0)
        {
            if (fabsf(slope->now_planning - slope->target) > slope->decrease_value)
                slope->out += slope->decrease_value;
            else
                slope->out = slope->target;
        }
    }
    else // now_planning == 0, 从静止起步, 用加速步长
    {
        if (slope->target > slope->now_planning) // 0值正向加速
        {
            if (fabsf(slope->now_planning - slope->target) > slope->increase_value)
                slope->out += slope->increase_value;
            else
                slope->out = slope->target;
        }
        else if (slope->target < slope->now_planning) // 0值负向加速
        {
            if (fabsf(slope->now_planning - slope->target) > slope->increase_value)
                slope->out -= slope->increase_value;
            else
                slope->out = slope->target;
        }
    }

    // 善后: 保存本次规划值, 作为下次的基准
    slope->now_planning = slope->out;
    return slope->out;
}
