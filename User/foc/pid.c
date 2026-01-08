#include "pid.h"

/**
 * @brief PI控制器初始化
 * @param pid PI控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param out_min 输出下限
 * @param out_max 输出上限
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;

    pid->error = 0.0f;
    pid->integral = 0.0f;

    pid->out = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;

    /* 积分限幅等于输出限幅 */
    pid->integral_max = out_max;
}

/**
 * @brief PI计算
 * @param pid PI控制器结构体指针
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @return 控制输出, 范围[out_min, out_max]
 */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback)
{
    /* 计算当前误差 */
    pid->error = setpoint - feedback;

    /* 比例项 */
    float proportional = pid->kp * pid->error;

    /* 积分项 - 带抗积分饱和 */
    float integral_increment = pid->ki * pid->error;

    /* 抗积分饱和：输出饱和时，阻止同向积分 */
    uint8_t saturated_high = (pid->out >= pid->out_max) && (integral_increment > 0);
    uint8_t saturated_low  = (pid->out <= pid->out_min) && (integral_increment < 0);
    
    if (!saturated_high && !saturated_low)
    {
        pid->integral += integral_increment;
    }

    /* 积分限幅 */
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    /* PI输出 = 比例 + 积分 */
    pid->out = proportional + pid->integral;

    /* 输出限幅 */
    if (pid->out > pid->out_max)
        pid->out = pid->out_max;
    else if (pid->out < pid->out_min)
        pid->out = pid->out_min;

    return pid->out;
}

/**
 * @brief PI复位
 * @param pid PI控制器结构体指针
 */
void pid_reset(pid_controller_t *pid)
{
    pid->error = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}
