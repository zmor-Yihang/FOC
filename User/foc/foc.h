#ifndef __FOC_H__
#define __FOC_H__

#include "config/type_config.h"
#include "clark_park.h"
#include "svpwm.h"
#include "pid.h"
#include "bsp/as5047.h"
#include "bsp/tim.h"
#include "bsp/adc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



// FOC控制函数
void foc_init(foc_t* handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed);

void foc_set_target(foc_t *handle, float target_Id, float target_Iq, float target_speed);



/* 电机零点对齐 */
void foc_alignment(foc_t *handle);

void foc_loop(foc_t *handle, float angle_el, abc_t *i_abc, uint16_t speed_rpm);

void foc_open_loop(dq_t u_dq, float angle_rad_el);

void foc_current_closed_loop(foc_t *handle, dq_t i_dq, float angle_el);




#endif /* __FOC_H__ */
