#ifndef __PID__H__ 
#define __PID__H__

#include "stm32g4xx_hal.h"

/* PID控制器结构体 */
typedef struct
{
    float kp; /* 比例系数 */
    float ki; /* 积分系数 */
    float kd; /* 微分系数, FOC 不用 */

    float error;      /* 当前误差 */
    float error_last; /* 上次误差 */
    float integral;   /* 积分项累加 */

    float out;          /* PID输出 */
    float out_max;      /* 输出限幅 */
    float out_min;      /* 输出下限 */
    float integral_max; /* 积分抗饱和限幅 */
} pid_controller_t;

/* PID控制器初始化 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_max, float out_min);

/* PID计算（位置式PID） */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback);

/* PID复位（清除积分和历史误差） */
void pid_reset(pid_controller_t *pid);

#endif /* __PID__H__ */
