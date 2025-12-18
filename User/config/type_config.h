#ifndef __TYPE_CONFIG_H__
#define __TYPE_CONFIG_H__

#include "stm32g4xx_hal.h"

typedef struct
{
    float a; // U
    float b; // V
    float c; // W
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
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数, FOC 不用

    float error;      // 当前误差
    float error_prev; // 上次误差
    float integral;   // 积分项累加

    float out;          // PID输出
    float out_max;      // 输出限幅
    float out_min;      // 输出下限
    float integral_max; // 积分抗饱和限幅
} pid_controller_t;

/* 传感器反馈结构体 */
typedef struct
{
    uint16_t raw_count; // 编码器/磁编码器原始值

    // 角度信息
    float angle_mechanical; // 机械角度 [0, 2PI]
    float angle_electrical; // 电角度 [0, 2PI] -> 用于Park变换
    float angle_offset;     // 电角度零点偏移量

    // 速度信息
    float speed_rad_s; // 实际转速 (弧度/秒)
    float speed_rpm;   // 实际转速 (RPM)

    // 辅助参数
    float sin_val; // sin(theta)
    float cos_val; // cos(theta)
} sensor_feedback_t;

/* 电机参数结构体 */
typedef struct
{
    int pole_pairs;      // 极对数
    float Rs;            // 相电阻 (Ohm)
    float Ls;            // 相电感 (H)
    float rated_current; // 额定电流 (A)
    float rated_voltage; // 额定电压 (V)
} motor_param_t;

// FOC 运行状态
typedef enum
{
    FOC_STATE_IDLE,        // 空闲/停止
    FOC_STATE_CALIBRATION, // 电流偏置校准
    FOC_STATE_ALIGNMENT,   // 角度对齐 (强拖)
    FOC_STATE_RUN,         // 闭环运行
    FOC_STATE_FAULT        // 故障保护
} foc_state_e;

// 核心控制对象
typedef struct
{
    // --- 状态管理 ---
    foc_state_e state;

    // 母线电压
    float dc_bus_voltage;

    // --- 硬件与参数 ---
    motor_param_t param;

    // --- 传感器反馈 ---
    sensor_feedback_t sensor;

    // --- 电流采样 (ADC 转换后的物理量) ---
    abc_t current_meas; // Ia, Ib, Ic (Measured)

    // --- 坐标变换变量 ---
    alphabeta_t i_alphabeta; // Clark变换后
    dq_t i_dq;        // Park变换后 (实际反馈值 Id, Iq)
    // --- 指令值 (Target) ---
    float target_speed;
    float target_Id; // 通常为0 (MTPA除外)
    float target_Iq; // 来自速度环输出

    // --- 控制器 ---
    pid_controller_t pid_id;    // D轴电流环
    pid_controller_t pid_iq;    // Q轴电流环
    pid_controller_t pid_speed; // 速度环

    // --- 输出计算 ---
    dq_t u_dq;        // 电流环PID输出电压)
    alphabeta_t u_alphabeta; // 反Park变换后
    abc_t duty_cycle;  // SVPWM计算出的占空比 (CCR值)

} foc_t;

#endif /* __TYPE_CONFIG_H__ */