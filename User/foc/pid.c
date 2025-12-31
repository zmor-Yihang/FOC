#include "pid.h"

/**
 * @brief PID控制器初始化
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param out_max 输出上限
 * @param out_min 输出下限
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_max, float out_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->integral = 0.0f;

    pid->out = 0.0f;
    pid->out_max = out_max;
    pid->out_min = out_min;

    /* 积分限幅默认设为输出限幅的2倍，以提供足够的积分作用范围 */
    pid->integral_max = (out_max > -out_min) ? out_max * 2.0f : (-out_min) * 2.0f;
}

/**
 * @brief PID计算, 位置式PID
 * @param pid PID控制器结构体指针
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @return 控制输出, 范围[out_min, out_max], 非归一化的值
 *
 * 位置式PID公式：
 * u(k) = Kp*e(k) + Ki*∑e(k) + Kd*[e(k) - e(k-1)]
 */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback)
{
    float proportional, derivative;

    /* 计算当前误差 */
    pid->error = setpoint - feedback;

    /* 比例项 */
    proportional = pid->kp * pid->error;

    /* 积分项 - 带抗积分饱和 */
    float integral_increment = pid->ki * pid->error;

    /* 检查输出是否饱和 */
    uint8_t output_saturated = 0;
    if (pid->out >= pid->out_max)
    {
        output_saturated = 1;
    }
    else if (pid->out <= pid->out_min)
    {
        output_saturated = 1;
    }

    /* 抗积分饱和：条件积分 */
    if (output_saturated)
    {
        /* 输出饱和时，只在误差方向有助于退出饱和时才积分 */
        if (pid->out >= pid->out_max && integral_increment < 0)
        {
            pid->integral += integral_increment;
        }
        else if (pid->out <= pid->out_min && integral_increment > 0)
        {
            pid->integral += integral_increment;
        }
    }
    else
    {
        /* 输出未饱和，正常积分 */
        pid->integral += integral_increment;
    }

    /* 积分限幅 */
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }

    /* 微分项 */
    derivative = pid->kd * (pid->error - pid->error_last);

    /* PID输出 = 比例 + 积分 + 微分 */
    pid->out = proportional + pid->integral + derivative;

    /* 输出限幅 */
    if (pid->out > pid->out_max)
    {
        pid->out = pid->out_max;
    }
    else if (pid->out < pid->out_min)
    {
        pid->out = pid->out_min;
    }

    /* 保存当前误差到历史 */
    pid->error_last = pid->error;

    return pid->out;
}

/**
 * @brief PID复位
 * @param pid PID控制器结构体指针
 * @note 清除积分项和历史误差，用于切换控制模式或重新启动时
 */
void pid_reset(pid_controller_t *pid)
{
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}

/**
 * @brief 设置PID参数
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void pid_set_params(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置输出限幅
 * @param pid PID控制器结构体指针
 * @param out_max 输出上限
 * @param out_min 输出下限
 */
void pid_set_limits(pid_controller_t *pid, float out_max, float out_min)
{
    pid->out_max = out_max;
    pid->out_min = out_min;
}
