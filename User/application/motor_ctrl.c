#include "motor_ctrl.h"

/* FOC 控制句柄 */
static foc_t foc_handle;

/* PID 控制器实例 */
static pid_controller_t pid_id;
static pid_controller_t pid_iq;
static pid_controller_t pid_speed;

/* 当前控制模式 */
static volatile ctrl_mode_t ctrl_mode = CTRL_MODE_IDLE;

/* 实际 dq 电流反馈 (打印用) */
static volatile dq_t i_dq_feedback = {0};

/* 速度斜坡控制 */
static float speed_ramp_target = 0.0f;  /* 最终目标转速 */
static float speed_ramp_current = 0.0f; /* 当前斜坡转速 */
#define SPEED_RAMP_STEP 1.0f            /* 斜坡增量 (rpm/周期, 10kHz下为10000rpm/s) */

/* FOC 主控制adc注入中断回调 - 10kHz */
static void motor_ctrl_callback(void)
{
    /* 更新编码器速度 */
    as5047_update_speed();

    /* 空闲模式不执行 */
    if (ctrl_mode == CTRL_MODE_IDLE)
        return;

    /* 开环模式单独处理 */
    if (ctrl_mode == CTRL_MODE_OPEN_LOOP)
    {
        foc_open_loop_run(&foc_handle, 200.0f, 1.0f);
        return;
    }

    /* 获取 ADC 采样值 */
    adc_values_t adc_values;
    adc1_get_injected_values(&adc_values);

    /* 计算电角度 */
    float angle_el = as5047_get_angle_rad() - foc_handle.angle_offset;

    /* Clark 变换: abc -> αβ */
    abc_t i_abc = {.a = adc_values.ia, .b = adc_values.ib, .c = adc_values.ic};
    alphabeta_t i_alphabeta = clark_transform(i_abc);

    /* Park 变换: αβ -> dq */
    dq_t i_dq = park_transform(i_alphabeta, angle_el);

    /* 保存电流反馈供打印 */
    i_dq_feedback = i_dq;

    /* 执行闭环控制 */
    if (ctrl_mode == CTRL_MODE_CURRENT)
    {
        foc_current_closed_loop_run(&foc_handle, i_dq, angle_el);
    }
    else if (ctrl_mode == CTRL_MODE_SPEED)
    {
        /* 斜坡加速/减速 */
        if (speed_ramp_current < speed_ramp_target)
        {
            speed_ramp_current += SPEED_RAMP_STEP;
            if (speed_ramp_current > speed_ramp_target)
                speed_ramp_current = speed_ramp_target;
        }
        else if (speed_ramp_current > speed_ramp_target)
        {
            speed_ramp_current -= SPEED_RAMP_STEP;
            if (speed_ramp_current < speed_ramp_target)
                speed_ramp_current = speed_ramp_target;
        }

        foc_handle.target_speed = speed_ramp_current;
        foc_speed_closed_loop_run(&foc_handle, i_dq, angle_el, as5047_get_speed_rpm());

        /* 减速完成后自动进入 IDLE */
        if (speed_ramp_target == 0.0f && fabsf(as5047_get_speed_rpm()) < 10.0f)
        {
            foc_handle.open_loop_angle_el = 0.0f;
            tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
            foc_closed_loop_stop(&foc_handle);
            ctrl_mode = CTRL_MODE_IDLE;
        }
    }
}

/* 状态进入函数 - 必须在关中断状态下调用 */
static void enter_idle(void)
{
    foc_handle.open_loop_angle_el = 0.0f;
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
    foc_closed_loop_stop(&foc_handle);
    ctrl_mode = CTRL_MODE_IDLE;
}

static void enter_open_loop(void)
{
    /* 从当前实际电角度开始，避免开环启动抖动 */
    foc_handle.open_loop_angle_el = as5047_get_angle_rad() - foc_handle.angle_offset;
    ctrl_mode = CTRL_MODE_OPEN_LOOP;
}

static void enter_current(void)
{   
    /* 用这个初始化 PID 积分项 */
    pid_reset(&pid_id);
    pid_reset(&pid_iq);
    foc_handle.target_id = 0.0f;
    foc_handle.target_iq = 0.5f;
    
    ctrl_mode = CTRL_MODE_CURRENT;
}


static void enter_speed(void)
{
    /* 无扰切换：保持电流环状态连续，不复位 pid_id/pid_iq */
    pid_speed.integral = foc_handle.target_iq;
    pid_speed.out = foc_handle.target_iq;
    pid_speed.error = 0.0f;
    speed_ramp_current = as5047_get_speed_rpm();
    speed_ramp_target = 3000.0f;
    ctrl_mode = CTRL_MODE_SPEED;
}

void motor_ctrl_bsp_init(void)
{
    HAL_Init();
    clock_init();
    usart1_init();

    led1_init();
    key_init();
    led2_init();

    as5047_init();
    tim3_init();
    tim1_init();
    adc1_init();
}

void motor_ctrl_init(void)
{
    /* 电流环 PI: Kp=0.017, Ki=0.002826, 输出限幅 ±U_DC/4 */
    pid_init(&pid_id, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);
    pid_init(&pid_iq, 0.017f, 0.002826f, -U_DC / 2.0f, U_DC / 2.0f);

    /* 速度环 PI: Kp=0.05, Ki=0.00002, 输出限幅 ±2A */
    pid_init(&pid_speed, 0.05f, 0.00002f, -4.0f, 4.0f);

    /* 初始化 FOC 控制句柄 */
    foc_init(&foc_handle, &pid_id, &pid_iq, &pid_speed);

    /* 注册 ADC 注入组中断回调 */
    adc1_register_injected_callback(motor_ctrl_callback);

    /* 电机零点对齐 */
    foc_alignment(&foc_handle);

    ctrl_mode = CTRL_MODE_IDLE;
}

void motor_ctrl_switch_mode(void)
{
    switch (ctrl_mode)
    {
    case CTRL_MODE_IDLE:
        enter_open_loop();
        printf("Mode: OPEN LOOP, 100RPM\n");
        break;

    case CTRL_MODE_OPEN_LOOP:
        enter_current();
        printf("Mode: CURRENT, Iq=0.5A\n");
        break;

    case CTRL_MODE_CURRENT:
        enter_speed();
        printf("Mode: SPEED, 3000RPM (bumpless)\n");
        break;

    case CTRL_MODE_SPEED:
        if (speed_ramp_target != 0.0f)
        {
            speed_ramp_target = 0.0f;
            printf("Mode: DECELERATING... (auto stop)\n");
        }
        else
        {
            /* 减速中再次按键，直接强制停止 */
            enter_idle();
            printf("Mode: IDLE (forced)\n");
        }
        break;
    }
}

void motor_ctrl_print_status(void)
{
    printf_period(500, "RPM, Id, Iq: %.1f, %.2f, %.2f\n", as5047_get_speed_rpm_lpf(), i_dq_feedback.d, i_dq_feedback.q);
}
