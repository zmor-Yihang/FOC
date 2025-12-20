#include "foc.h"

void foc_init(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq, pid_controller_t *pid_speed)
{
    handle->pole_pairs = MOTOR_POLES; // 设置电机极对数
    handle->target_speed = 0;
    handle->target_Id = 0;
    handle->target_Iq = 0;

    /* 初始化电流采样值 */
    handle->i_abc.a = 0.0f;
    handle->i_abc.b = 0.0f;
    handle->i_abc.c = 0.0f;

    /* 初始化Clark变换后的电流值 */
    handle->i_alphabeta.alpha = 0.0f;
    handle->i_alphabeta.beta = 0.0f;

    /* 初始化Park变换后的电流值 */
    handle->i_dq.d = 0.0f;
    handle->i_dq.q = 0.0f;

    /* 初始化电压控制量 */
    handle->v_d = 0.0f;
    handle->v_q = 0.0f;

    /* 初始化Park变换后的电压值 */
    handle->v_alpha = 0.0f;
    handle->v_beta = 0.0f;

    /* 初始化零点偏移 */
    handle->zero_offset = 0.0f;

    /* 初始化母线电压为默认值 */
    handle->vbus = 13.15f; // 默认13.15V

    /* 初始化PID控制器指针 */
    handle->pid_id = pid_id;
    handle->pid_iq = pid_iq;
    handle->pid_speed = pid_speed;

    /* 初始化占空比为0 */
    handle->duty_cycle.a = 0.0f;
    handle->duty_cycle.b = 0.0f;
    handle->duty_cycle.c = 0.0f;
}

void foc_set_target_speed(foc_t *handle, float speed_rpm)
{
    handle->target_speed = speed_rpm;
}

void foc_set_target_currents(foc_t *handle, float Id, float Iq)
{
    handle->target_Id = Id;
    handle->target_Iq = Iq;
}

void foc_alignment(foc_t *handle)
{

    printf("Motor Alignment Start...\r\n");

    /* 读取母线电压 */
    adc1_start_regular_dma();
    HAL_Delay(100); // 等待DMA转换完成
    
    adc_values_t adc_vals = {0};
    adc1_value_convert((uint16_t *)adc_regular_buf, &adc_vals);
    handle->vbus = adc_vals.udc;

    printf("Bus Voltage: %.2fV\r\n", handle->vbus);

    /* 1. 准备阶段：设置目标电压矢量指向 D 轴 (电角度 0 度) */
    /* 注意：电压不要给太大，防止电机过热烧毁！通常给母线电压的 10%~20% */
    /* 归一化电压：实际电压 / 母线电压 */
    handle->v_d = 2.0f / handle->vbus;
    handle->v_q = 0.0f;

    printf("Alignment voltage: %.2fV (normalized: %.3f)\r\n", 2.0f, handle->v_d);

    /* 强制设定电角度为 0，用于 Park 逆变换 */
    /* 因为我们要产生一个固定在 0 度的磁场 */
    float align_angle = 0.0f;

    /* 2. 执行阶段：持续输出一段时间，让转子对齐到电角度0位置 */
    int align_steps = 2000 / 2; // 总对齐时间除以每步延时

    for (int i = 0; i < align_steps; i++)
    {
        /* 反 Park 变换 (输入 Vd, Vq, 角度=0) -> 得到 V_alpha, V_beta */
        dq_t v_dq = {handle->v_d, handle->v_q};
        alphabeta_t v_alphabeta = ipark_transform(v_dq, align_angle);
        handle->v_alpha = v_alphabeta.alpha;
        handle->v_beta = v_alphabeta.beta;

        /* 反 Clark 变换 -> 得到三相电压 */
        abc_t v_abc = iclark_transform(v_alphabeta);

        /* SVPWM 计算 -> 得到三相占空比 */
        handle->duty_cycle = svpwm_update(v_abc);

        /* 调用底层驱动设置 PWM 寄存器 */
        tim1_set_pwm_duty(handle->duty_cycle.a, handle->duty_cycle.b, handle->duty_cycle.c);

        HAL_Delay(2);
    }

    /* 3. 读取阶段：记录偏移量 */
    /* 此时电机应该已经对齐到电角度 0 度位置 */
    /* 读取编码器的机械角度原始值 */
    float raw_angle = as5047_read_angle_rad();

    handle->zero_offset = raw_angle;

    printf("Alignment Done. Offset = %.4f rad\r\n", handle->zero_offset);

    /* 4. 结束阶段：归零输出 */
    handle->v_d = 0.0f;
    handle->v_q = 0.0f;
    tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
}

void foc_loop(foc_t *handle, float angle_el, abc_t *i_abc, uint16_t speed_rpm)
{
    // TODO: 实现FOC闭环控制逻辑
    // 这里应该包含完整的FOC控制流程：
    // 1. 电流环控制（Id, Iq控制）
    // 2. 速度环控制（如果需要速度控制）
    // 3. Park/Clark变换
    // 4. SVPWM调制
    // 5. PWM输出
}

void foc_open_loop(foc_t *handle, float voltage, float angle_rad_el)
{
    /* 限制电压幅值 */
    if (voltage > 1.0f)
        voltage = 1.0f;
    if (voltage < 0.0f)
        voltage = 0.0f;

    /* 使用反Park变换计算Alpha-Beta电压 */
    dq_t v_dq = {0.0f, voltage}; /* Vd=0, Vq=voltage */
    alphabeta_t v_ab = ipark_transform(v_dq, angle_rad_el);

    /* 使用反Clark变换计算ABC电压 */
    abc_t v_abc = iclark_transform(v_ab);

    /* SVPWM调制 */
    abc_t duty = svpwm_update(v_abc);

    /* 输出PWM */
    tim1_set_pwm_duty(duty.a, duty.b, duty.c);
}

void foc_simulate_rotation(foc_t *handle, float voltage, float speed_rpm, uint32_t duration_ms)
{
    uint32_t start_time, current_time;
    float angle_mechanical = 0.0f;
    float angle_electrical = 0.0f;
    float speed_rad_per_sec;
    
    /* 转换转速单位：RPM -> rad/s */
    speed_rad_per_sec = speed_rpm * 2.0f * 3.14159265359f / 60.0f;
    
    /* 获取开始时间 */
    start_time = HAL_GetTick();
    
    /* 旋转控制循环 */
    while (1)
    {
        /* 获取当前时间 */
        current_time = HAL_GetTick();
        
        /* 检查是否超时 */
        if ((current_time - start_time) >= duration_ms)
        {
            break;
        }
        
        /* 更新机械角度 (角度自增) */
        angle_mechanical += speed_rad_per_sec * 0.001f;  // 假设1ms执行一次
        
        /* 保持角度在0-2π范围内 */
        if (angle_mechanical >= 2.0f * 3.14159265359f)
        {
            angle_mechanical -= 2.0f * 3.14159265359f;
        }
        
        /* 计算电角度 */
        angle_electrical = angle_mechanical * handle->pole_pairs;
        
        /* 减去零点偏移 (如果已知) */
        if (handle->zero_offset != 0.0f)
        {
            angle_electrical -= handle->zero_offset;
            
            /* 保持角度在0-2π范围内 */
            if (angle_electrical < 0.0f)
            {
                angle_electrical += 2.0f * 3.14159265359f;
            }
            else if (angle_electrical >= 2.0f * 3.14159265359f)
            {
                angle_electrical -= 2.0f * 3.14159265359f;
            }
        }
        
        /* 开环控制 */
        foc_open_loop(handle, voltage, angle_electrical);
        
        /* 延时1ms */
        HAL_Delay(1);
    }
    
    /* 停止电机 */
    tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
}