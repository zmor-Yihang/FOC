#ifndef __FOC_H__
#define __FOC_H__

#include "./config/type_config.h"
#include "./bsp/adc.h"
#include "./bsp/as5047.h"
#include "./bsp/tim.h"
#include "./foc/clark_park.h"
#include "./foc/svpwm.h"
#include "./foc/pid.h"
#include <string.h>

/* ==================== FOC 参数配置 ==================== */

/* 电流滤波系数 (一阶低通滤波器, 0~1, 越小滤波越强) */
#define FOC_CURRENT_FILTER_ALPHA    0.2f

/* 速度滤波系数 */
#define FOC_SPEED_FILTER_ALPHA      0.1f

/* 控制周期 (秒) - 与TIM1 PWM频率对应 */
#define FOC_CONTROL_PERIOD          0.0001f     /* 10kHz -> 100us */

/* 速度环分频系数 (速度环频率 = 电流环频率 / 分频系数) */
#define FOC_SPEED_LOOP_DIVIDER      10          /* 速度环1kHz */

/* 默认PID参数 */
#define FOC_PID_ID_KP               0.5f
#define FOC_PID_ID_KI               0.01f
#define FOC_PID_ID_KD               0.0f
#define FOC_PID_ID_MAX              1.0f        /* 归一化电压输出 */
#define FOC_PID_ID_MIN              (-1.0f)

#define FOC_PID_IQ_KP               0.5f
#define FOC_PID_IQ_KI               0.01f
#define FOC_PID_IQ_KD               0.0f
#define FOC_PID_IQ_MAX              1.0f
#define FOC_PID_IQ_MIN              (-1.0f)

#define FOC_PID_SPEED_KP            0.1f
#define FOC_PID_SPEED_KI            0.001f
#define FOC_PID_SPEED_KD            0.0f
#define FOC_PID_SPEED_MAX           5.0f        /* 最大Iq电流参考 (A) */
#define FOC_PID_SPEED_MIN           (-5.0f)

/* ==================== FOC 状态枚举 ==================== */

/* FOC运行状态 */
typedef enum
{
    FOC_STATE_IDLE = 0,         /* 空闲状态 */
    FOC_STATE_CALIBRATING,      /* 校准中 */
    FOC_STATE_READY,            /* 就绪，等待启动 */
    FOC_STATE_RUNNING,          /* 正常运行 */
    FOC_STATE_ERROR             /* 错误状态 */
} foc_state_e;

/* FOC控制模式 */
typedef enum
{
    FOC_MODE_TORQUE = 0,        /* 力矩控制模式 (仅电流环) */
    FOC_MODE_SPEED,             /* 速度控制模式 (速度环 + 电流环) */
    FOC_MODE_POSITION,          /* 位置控制模式 (位置环 + 速度环 + 电流环) */
    FOC_MODE_OPENLOOP           /* 开环模式 (用于调试) */
} foc_mode_e;

/* 错误码 */
typedef enum
{
    FOC_ERR_NONE = 0,           /* 无错误 */
    FOC_ERR_OVERCURRENT,        /* 过流 */
    FOC_ERR_OVERVOLTAGE,        /* 过压 */
    FOC_ERR_UNDERVOLTAGE,       /* 欠压 */
    FOC_ERR_ENCODER,            /* 编码器故障 */
    FOC_ERR_CALIBRATION         /* 校准失败 */
} foc_error_e;

/* ==================== FOC 扩展结构体 ==================== */

/* 电流采样滤波器 */
typedef struct
{
    abc_t raw;                  /* 原始采样值 */
    abc_t filtered;             /* 滤波后的值 */
    float alpha;                /* 滤波系数 */
} current_filter_t;

/* FOC 扩展控制结构体 */
typedef struct
{
    /* 基础FOC结构体 */
    foc_t core;
    
    /* ADC值结构体 */
    adc_values_t adc_values;
    
    /* 电流滤波器 */
    current_filter_t current_filter;
    
    /* 电气角度 */
    float electrical_angle;     /* 电气角度 (rad) */
    float mechanical_angle;     /* 机械角度 (rad) */
    float speed_rpm;            /* 速度 (RPM) */
    float speed_rad_s;          /* 角速度 (rad/s) */
    float speed_filtered;       /* 滤波后速度 */
    
    /* 状态与模式 */
    foc_state_e state;
    foc_mode_e mode;
    foc_error_e error;
    
    /* 速度环分频计数器 */
    uint16_t speed_loop_cnt;
    
    /* 开环控制参数 (调试用) */
    float openloop_angle;       /* 开环角度 */
    float openloop_speed;       /* 开环角速度 (rad/s) */
    float openloop_voltage;     /* 开环电压幅值 */
    
} foc_controller_t;

/* ==================== 全局FOC对象 ==================== */

extern foc_controller_t g_foc;

/* ==================== FOC API 函数声明 ==================== */

/**
 * @brief  FOC初始化
 * @param  pole_pairs: 电机极对数
 * @retval None
 */
void foc_init(int pole_pairs);

/**
 * @brief  设置电机参数
 * @param  Rs: 相电阻 (Ohm)
 * @param  Ls: 相电感 (H)
 * @param  rated_current: 额定电流 (A)
 * @param  rated_voltage: 额定电压 (V)
 */
void foc_set_motor_params(float Rs, float Ls, float rated_current, float rated_voltage);

/**
 * @brief  设置电流环PID参数
 * @param  kp_d, ki_d, kd_d: D轴PID参数
 * @param  kp_q, ki_q, kd_q: Q轴PID参数
 */
void foc_set_current_pid(float kp_d, float ki_d, float kd_d,
                         float kp_q, float ki_q, float kd_q);

/**
 * @brief  设置速度环PID参数
 * @param  kp, ki, kd: PID参数
 */
void foc_set_speed_pid(float kp, float ki, float kd);

/**
 * @brief  ADC零点校准
 * @note   启动前调用，电机静止状态
 */
void foc_calibrate_adc(void);

/**
 * @brief  设置控制模式
 * @param  mode: 控制模式
 */
void foc_set_mode(foc_mode_e mode);

/**
 * @brief  设置目标速度
 * @param  speed_rpm: 目标速度 (RPM)
 */
void foc_set_target_speed(float speed_rpm);

/**
 * @brief  设置目标力矩 (电流)
 * @param  iq: 目标Q轴电流 (A)
 */
void foc_set_target_torque(float iq);

/**
 * @brief  启动FOC
 */
void foc_start(void);

/**
 * @brief  停止FOC
 */
void foc_stop(void);

/**
 * @brief  紧急停止 (立即停止PWM输出)
 */
void foc_emergency_stop(void);

/**
 * @brief  FOC主循环 - 在TIM1中断中调用
 * @note   执行电流采样、坐标变换、电流环控制、SVPWM输出
 */
void foc_loop(void);

/**
 * @brief  速度环控制 - 由foc_loop根据分频调用
 */
void foc_speed_loop(void);

/**
 * @brief  电流环控制
 */
void foc_current_loop(void);

/**
 * @brief  电流采样和滤波
 */
void foc_sample_current(void);

/**
 * @brief  电压输出计算并设置PWM
 */
void foc_output_voltage(void);

/**
 * @brief  获取当前状态
 * @retval FOC状态
 */
foc_state_e foc_get_state(void);

/**
 * @brief  获取错误码
 * @retval 错误码
 */
foc_error_e foc_get_error(void);

/**
 * @brief  清除错误
 */
void foc_clear_error(void);

/**
 * @brief  开环控制 (调试用)
 * @param  voltage: 电压幅值 (归一化 0~1)
 * @param  speed: 角速度 (rad/s)
 */
void foc_openloop_run(float voltage, float speed);

#endif /* __FOC_H__ */
