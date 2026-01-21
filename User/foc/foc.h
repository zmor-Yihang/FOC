#ifndef __FOC_H__
#define __FOC_H__

#include "stm32g4xx_hal.h"
#include "clark_park.h"
#include "svpwm.h"
#include "pid.h"
#include "bsp/as5047.h"
#include "bsp/tim.h"
#include "bsp/adc.h"

/* 电机参数 */
#define U_DC 12.0f        /* 直流母线电压 (V) */

/* FOC 核心控制对象 */
typedef struct
{
    /* 电流采样值 */
    abc_t i_abc;

    /* 目标值 */
    float target_speed;
    float target_id;
    float target_iq;

    /* 电压控制量 */
    float v_d_out; /* D轴电压输出 */
    float v_q_out; /* Q轴电压输出 */
    float i_q_out; /* Q轴电流输出 (速度环) */

    /* PID控制器 */
    pid_controller_t *pid_id;
    pid_controller_t *pid_iq;
    pid_controller_t *pid_speed;

    /* 输出占空比 */
    abc_t duty_cycle;

    /* 编码器零点偏移 */
    float angle_offset;

    /* 开环运行角度 */
    float open_loop_angle_el;
} foc_t;

/* FOC 控制函数 */
void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed);
void foc_alignment(foc_t *handle);

/* 开环控制 */
void foc_open_loop_run(foc_t *handle, float speed_rpm, float voltage_q);
void foc_if_current_run(foc_t *handle, dq_t i_dq, float speed_rpm, float current_q);

/* 闭环控制 */
void foc_current_closed_loop_run(foc_t *handle, dq_t i_dq, float angle_el);
void foc_speed_closed_loop_run(foc_t *handle, dq_t i_dq, float angle_el, float speed_rpm);
void foc_closed_loop_stop(foc_t *handle);

// 目标值设置函数
void foc_set_target_id(foc_t *handle, float id);
void foc_set_target_iq(foc_t *handle, float iq);
void foc_set_target_speed(foc_t *handle, float speed_rpm);

#endif /* __FOC_H__ */
