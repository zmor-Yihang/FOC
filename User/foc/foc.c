#include "foc.h"
#include <math.h>

/* ==================== 全局FOC控制对象 ==================== */
foc_controller_t g_foc;

/* ==================== 内部辅助函数 ==================== */

/**
 * @brief  保护检测
 */
static void foc_check_protection(void)
{
    float max_current = g_foc.core.param.rated_current * 1.5f; /* 1.5倍额定电流 */

    /* 过流保护 */
    if (fabsf(g_foc.core.current_meas.a) > max_current ||
        fabsf(g_foc.core.current_meas.b) > max_current ||
        fabsf(g_foc.core.current_meas.c) > max_current)
    {
        g_foc.error = FOC_ERR_OVERCURRENT;
        g_foc.state = FOC_STATE_ERROR;
        return;
    }

    /* 过压保护 */
    if (g_foc.core.dc_bus_voltage > g_foc.core.param.rated_voltage * 1.2f)
    {
        g_foc.error = FOC_ERR_OVERVOLTAGE;
        g_foc.state = FOC_STATE_ERROR;
        return;
    }

    /* 欠压保护 */
    if (g_foc.core.dc_bus_voltage < g_foc.core.param.rated_voltage * 0.5f &&
        g_foc.core.dc_bus_voltage > 1.0f) /* 防止未上电时误报 */
    {
        g_foc.error = FOC_ERR_UNDERVOLTAGE;
        g_foc.state = FOC_STATE_ERROR;
        return;
    }
}


/**
 * @brief  启动FOC
 */
void foc_start(void)
{
    if (g_foc.state == FOC_STATE_READY || g_foc.state == FOC_STATE_IDLE)
    {
        /* 复位所有控制器 */
        foc_reset_controllers();

        /* 清零开环角度 */
        g_foc.openloop_angle = 0.0f;

        /* 切换到运行状态 */
        g_foc.state = FOC_STATE_RUNNING;
    }
}

/**
 * @brief  停止FOC
 */
void foc_stop(void)
{
    /* 设置占空比为50% (零输出) */
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);

    /* 复位控制器 */
    foc_reset_controllers();

    /* 清零目标值 */
    g_foc.core.target_speed = 0.0f;
    g_foc.core.target_Iq = 0.0f;
    g_foc.core.target_Id = 0.0f;

    g_foc.state = FOC_STATE_READY;
}

/**
 * @brief  紧急停止
 */
void foc_emergency_stop(void)
{
    /* 立即停止PWM输出 */
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);

    g_foc.state = FOC_STATE_IDLE;
}


/**
 * @brief  速度环控制
 */
void foc_speed_loop(void)
{
    /* 读取速度 */
    g_foc.speed_rpm = as5047_read_speed_rpm();
    g_foc.speed_rad_s = as5047_read_speed_rad_s();

    /* 速度滤波 */
    g_foc.speed_filtered = FOC_SPEED_FILTER_ALPHA * g_foc.speed_rpm +
                           (1.0f - FOC_SPEED_FILTER_ALPHA) * g_foc.speed_filtered;

    /* 速度环PID计算，输出为Iq参考 */
    g_foc.core.target_Iq = pid_calculate(&g_foc.core.pid_speed,
                                         g_foc.core.target_speed,
                                         g_foc.speed_filtered);

    /* Id参考通常为0 (非弱磁控制) */
    g_foc.core.target_Id = 0.0f;
}

/* ==================== 电流环控制 ==================== */

/**
 * @brief  电流环控制
 */
void foc_current_loop(void)
{
    /* D轴电流环 */
    g_foc.core.u_dq.d = pid_calculate(&g_foc.core.pid_id,
                                      g_foc.core.target_Id,
                                      g_foc.core.i_dq.d);

    /* Q轴电流环 */
    g_foc.core.u_dq.q = pid_calculate(&g_foc.core.pid_iq,
                                      g_foc.core.target_Iq,
                                      g_foc.core.i_dq.q);

    /* 电压幅值限制 (防止过调制) */
    float u_abs = sqrtf(g_foc.core.u_dq.d * g_foc.core.u_dq.d +
                        g_foc.core.u_dq.q * g_foc.core.u_dq.q);

    if (u_abs > 1.0f)
    {
        g_foc.core.u_dq.d /= u_abs;
        g_foc.core.u_dq.q /= u_abs;
    }
}


/* ==================== FOC 主循环 ==================== */

/**
 * @brief  FOC主循环 - 在TIM1中断中调用
 */
void foc_loop(void)
{
    /* 1. 电流采样和滤波 */
    foc_sample_current();

    /* 2. 保护检测 */
    foc_check_protection();
    
    if (g_foc.state == FOC_STATE_ERROR)
    {
        foc_emergency_stop();
        return;
    }


    foc_coordinate_transform();

    /* 速度环分频执行 */
    g_foc.speed_loop_cnt++;
    if (g_foc.speed_loop_cnt >= FOC_SPEED_LOOP_DIVIDER)
    {
        g_foc.speed_loop_cnt = 0;
        foc_speed_loop();
    }

    foc_current_loop();
    foc_output_voltage();
}


