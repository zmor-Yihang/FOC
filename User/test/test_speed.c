#include "test_speed.h"
#include "foc/foc.h"
#include "foc/pid.h"
#include "foc/smo.h"
#include "bsp/adc.h"
#include "bsp/as5047.h"
#include "bsp/tim.h"
#include "utils/print.h"
#include <stdio.h>
#include <math.h>

/* FOC 控制句柄 */
static foc_t foc;

/* PID 控制器 */
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

/* SMO 观测器 */
static smo_t smo;

/* 测试参数 */
static float target_speed_rpm = 0.0f; /* 目标转速 */
static volatile uint8_t test_running = 0;

/* 电机参数 */
#define MOTOR_RS 0.12f     /* 定子电阻 (Ω) */
#define MOTOR_LS 0.0003f   /* 定子电感 (H) */
#define MOTOR_POLES 7.0f   /* 极对数 */
#define CONTROL_TS 0.0001f /* 控制周期 (s) - 10kHz */

/**
 * @brief FOC控制回调函数 - 在ADC注入中断中调用 (10kHz)
 */
static void test_speed_callback(void)
{
    if (!test_running)
        return;

    /* 更新编码器速度 */
    as5047_update_speed();

    /* 获取ADC采样电流 */
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    /* 获取电角度 */
    float angle_el = as5047_get_angle_rad() - foc.angle_offset;

    /* Clark变换: abc -> αβ */
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    /* Park变换: αβ -> dq */
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    /* 获取实际转速 */
    float actual_speed_rpm = as5047_get_speed_rpm();

    /* 三闭环控制 */
    foc.target_speed = target_speed_rpm;
    foc_speed_closed_loop_run(&foc, i_dq, angle_el, actual_speed_rpm);

    /* === SMO观测器更新 === */
    /* 输入实测电流 αβ */
    smo.i_alpha = i_alphabeta.alpha;
    smo.i_beta = i_alphabeta.beta;

    /* 输入控制电压 αβ (从dq反变换得到) */
    /* 注意：需要使用实际的母线电压和占空比计算 */
    dq_t v_dq = {.d = foc.v_d_out, .q = foc.v_q_out};
    alphabeta_t v_alphabeta = ipark_transform(v_dq, angle_el);
    smo.u_alpha = v_alphabeta.alpha;
    smo.u_beta = v_alphabeta.beta;

    /* 执行SMO估算 */
    smo_estimate(&smo);
}

/**
 * @brief 初始化并启动速度闭环测试
 * @param target_rpm 目标转速 (RPM)
 *
 * 使用示例：
 *   test_speed_loop_init(1000.0f);  // 启动，目标1000 RPM
 *   while(1) {
 *       test_speed_loop_print_status();  // 定期打印状态
 *       HAL_Delay(100);
 *   }
 */
void test_speed_loop_init(float target_rpm)
{
    printf("\r\n========== 速度闭环测试 (含SMO观测) ==========\r\n");

    /* 初始化PID控制器 */
    /* 电流环PI: Kp=0.017, Ki=0.002826, 输出限幅±6V */
    pid_init(&pid_id, 0.017f, 0.002826f, -6.0f, 6.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -6.0f, 6.0f);

    /* 速度环PI: Kp=0.05, Ki=0.00002, 输出限幅±4A */
    pid_init(&pid_speed, 0.05f, 0.00002f, -4.0f, 4.0f);

    /* 初始化FOC */
    foc_init(&foc, &pid_id, &pid_iq, &pid_speed);

    /* 电机零点对齐 */
    printf("电机零点对齐中...\r\n");
    foc_alignment(&foc);
    printf("对齐完成，零点偏移: %.4f rad\r\n", foc.angle_offset);

    /* 初始化SMO观测器 */
    /* 参数: rs, ls, poles, ts, k_slide, k_lpf, boundary, fc */
    float voltage_lim = 12.0f; /* 假设母线电压12V，根据实际调整 */
    smo_init(&smo, MOTOR_RS, MOTOR_LS, MOTOR_POLES, CONTROL_TS,
             0.17f,              /* k_slide: 滑模增益系数 */
             0.06f,              /* k_lpf: 低通滤波系数 (保留6%旧值) */
             voltage_lim / 4.0f, /* boundary: 电流误差限幅值 */
             200.0f);            /* fc: PLL截止频率 (Hz) */

    /* 用编码器角度初始化SMO */
    smo.theta_est = as5047_get_angle_rad() - foc.angle_offset;
    
    printf("SMO参数: Rs=%.2f Ω, Ls=%.4f H, Poles=%.0f\r\n",
           MOTOR_RS, MOTOR_LS, MOTOR_POLES);
    printf("SMO参数: Kslide=%.2f, Kslf=%.2f, E0=%.2f\r\n",
           0.17f, 0.06f, voltage_lim / 4.0f);
    printf("SMO初始角度: %.4f rad\r\n", smo.theta_est);

    /* 注册ADC中断回调 */
    adc1_register_injected_callback(test_speed_callback);

    /* 设置目标转速 */
    target_speed_rpm = target_rpm;
    printf("目标转速: %.1f RPM\r\n", target_speed_rpm);
    printf("速度闭环已启动\r\n\r\n");

    /* 启动测试 */
    test_running = 1;
}

/**
 * @brief 打印状态信息 - 对比编码器和SMO观测结果
 *
 * 建议在主循环中每100ms调用一次
 */
void test_speed_loop_print_status(void)
{
    if (!test_running)
        return;

    /* 编码器数据 */
    float encoder_rpm = as5047_get_speed_rpm_lpf();
    float encoder_angle = as5047_get_angle_rad() - foc.angle_offset;

    /* 角度归一化到 [0, 2π] */
    const float TWO_PI = 2.0f * 3.14159265f;
    while (encoder_angle >= TWO_PI)
        encoder_angle -= TWO_PI;
    while (encoder_angle < 0.0f)
        encoder_angle += TWO_PI;

    /* SMO观测数据 */
    float smo_rpm = smo_get_speed(&smo);
    float smo_angle = smo_get_angle(&smo);
    
    /* SMO内部状态 - 用于调试 */
    float bemf_alpha = smo_get_bemf_alpha(&smo);
    float bemf_beta = smo_get_bemf_beta(&smo);

    /* 计算误差 */
    float angle_error = encoder_angle - smo_angle;

    /* 角度误差归一化到 [-π, π] */
    while (angle_error > 3.14159265f)
        angle_error -= TWO_PI;
    while (angle_error < -3.14159265f)
        angle_error += TWO_PI;

    printf(" %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
           encoder_rpm, encoder_angle, smo_rpm, smo_angle, bemf_alpha, bemf_beta);
}
