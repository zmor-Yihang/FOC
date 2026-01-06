#ifndef __PID__H__ 
#define __PID__H__

#include "./config/type_config.h"

/* PID控制器初始化 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_max, float out_min);

/* PID计算（位置式PID） */
float pid_calculate(pid_controller_t *pid, float setpoint, float feedback);

/* PID复位（清除积分和历史误差） */
void pid_reset(pid_controller_t *pid);

#endif /* __PID__H__ */
