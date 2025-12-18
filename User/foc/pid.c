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
    pid->error_prev = 0.0f;
    pid->integral = 0.0f;

    pid->out = 0.0f;
    pid->out_max = out_max;
    pid->out_min = out_min;

    /* 积分限幅默认设为输出限幅 */
    pid->integral_max = out_max;
}

/**
 * @brief PID计算（位置式PID）
 * @param pid PID控制器结构体指针
 * @param setpoint 设定值
 * @param feedback 反馈值
 * @return 控制输出
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

    /* 积分项（带抗饱和） */
    pid->integral += pid->ki * pid->error;

    /* 积分限幅（抗积分饱和） */
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < -pid->integral_max)
    {
        pid->integral = -pid->integral_max;
    }

    /* 微分项 */
    derivative = pid->kd * (pid->error - pid->error_prev);

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
    pid->error_prev = pid->error;

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
    pid->error_prev = 0.0f;
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
