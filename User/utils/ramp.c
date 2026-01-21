#include "ramp.h"

/**
 * @brief 通用斜坡函数
 * @param current 当前值
 * @param target  目标值
 * @param rate    变化速率（单位/秒）
 * @param dt      时间步长（秒）
 * @return 更新后的当前值
 */
float ramp_update(float current, float target, float rate, float dt)
{
    float delta = target - current;
    float max_step = rate * dt; // 本次允许的最大变化量

    if (delta > max_step)
    {
        return current + max_step; // 正向加速
    }
    else if (delta < -max_step)
    {
        return current - max_step; // 负向减速
    }
    else
    {
        return target; // 已到达目标
    }
}
