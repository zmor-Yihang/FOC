#ifndef __TYPE_CONFIG_H__
#define __TYPE_CONFIG_H__

#include "stm32g4xx_hal.h"

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

/* ADC 参数结构体 */
typedef struct
{
    float ia;
    float ib;
    float ic;
    float udc;

    float ia_offset;
    float ib_offset;
    float ic_offset;
} adc_values_t;

/* 核心控制对象 */
typedef struct
{
    /* 电机极对数 */
    uint8_t pole_pairs;

    /* 电流采样值Ia, Ib, Ic */
    abc_t i_abc;
    alphabeta_t i_alphabeta; /* clark变换后 */
    dq_t i_dq;               /* park变换后 */

    /* 目标值 */
    float target_speed;
    float target_Id;
    float target_Iq;

    /* 电压控制量 */
    float v_d;           /* D轴目标电压 */
    float v_q;           /* Q轴目标电压 */
    float v_alpha;       /* Alpha轴电压 (反Park变换后) */
    float v_beta;        /* Beta轴电压 (反Park变换后) */

    /* 零点偏移 */
    float zero_offset;   /* 编码器零点偏移角度 (rad) */

    /* 母线电压 */
    float vbus;          /* 母线电压 (V) */

    /* PID控制器 */
    pid_controller_t *pid_id;    /* D轴电流环 */
    pid_controller_t *pid_iq;    /* Q轴电流环 */
    pid_controller_t *pid_speed; /* 速度环 */

    /* 输出占空比 */
    abc_t duty_cycle; /* SVPWM计算出的占空比 */

} foc_t;

#endif /* __TYPE_CONFIG_H__ */