#ifndef __PID__H__ 
#define __PID__H__

#include "stm32g4xx_hal.h"

/* PI控制器结构体 */
typedef struct
{
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */

    float error;        /* 当前误差 */
    float integral;     /* 积分项累加 */

    float out;          /* PI输出 */
    float out_min;      /* 输出下限 */
    float out_max;      /* 输出上限 */
    float integral_max; /* 积分抗饱和限幅 */
} pid_controller_t;

/* PI控制器初始化 */
void pid_init(pid_controller_t *pid, float kp, float ki, float out_min, float out_max);

/* PI计算 */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback);

/* PI复位 */
void pid_reset(pid_controller_t *pid);

#endif /* __PID__H__ */
