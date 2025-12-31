#ifndef __TYPE_CONFIG_H__
#define __TYPE_CONFIG_H__

#include "stm32g4xx_hal.h"

#define MOTOR_POLE_PAIR 7 /* 电机极对数 */

#define U_DC 12.0f /* 直流母线电压 (V) */

typedef struct
{
    float a; /* U */
    float b; /* V */
    float c; /* W */
} abc_t;

typedef struct
{
    float alpha;
    float beta;
} alphabeta_t;

typedef struct
{
    float d;
    float q;
} dq_t;

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

/* 一阶低通滤波器结构体 */
typedef struct
{
    float alpha;      /* 滤波系数 (0-1) */
    float output;     /* 滤波器输出 */
    float initialized; /* 初始化标志 */
} lpf_t;

/* ADC 参数结构体 */
typedef struct
{
    float ia;
    float ib;
    float ic;
    float udc;
} adc_values_t;

/* 核心控制对象 */
typedef struct
{
    /* 电流采样值Ia, Ib, Ic */
    abc_t i_abc;

    /* 目标值 */
    float target_speed;
    float target_Id;
    float target_Iq;

    /* 电压控制量 */
    float v_d_out; /* D轴目标电压, 电流环输出，实际值 */
    float v_q_out; /* Q轴目标电压, 电流环输出，实际值 */
    float i_q_out; /* Q轴目标电流， 转速环输出 */

    /* PID控制器 */
    pid_controller_t *pid_id;    /* D轴电流环 */
    pid_controller_t *pid_iq;    /* Q轴电流环 */
    pid_controller_t *pid_speed; /* 速度环 */

    /* 电流滤波器 */
    lpf_t lpf_id; /* D轴电流低通滤波器 */
    lpf_t lpf_iq; /* Q轴电流低通滤波器 */

    /* 输出占空比 */
    abc_t duty_cycle; /* SVPWM计算出的占空比 */

    /* 编码器零点偏移 */
    float angle_offset; /* 编码器零点偏移角 (rad)，用于计算实际电角度 */
} foc_t;

#endif /* __TYPE_CONFIG_H__ */