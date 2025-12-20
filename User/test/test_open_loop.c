#include "test_open_loop.h"

/**
 * @brief FOC开环控制测试函数
 * 测试功能：
 * 1. 电机对齐测试
 * 2. 开环低速旋转测试
 * 3. 开环变速测试
 * 4. 开环正反转测试
 */
void test_open_loop(void)
{
    foc_t hfoc;
    pid_controller_t pid_id, pid_iq, pid_speed;
    
    /* 初始化PID控制器 */
    pid_init(&pid_id, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f);
    pid_init(&pid_iq, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f); 
    pid_init(&pid_speed, 1.0f, 0.1f, 0.0f, 5.0f, -5.0f);
    
    /* 初始化FOC控制器 */
    foc_init(&hfoc, &pid_id, &pid_iq, &pid_speed);
    
    vofa_print(&huart1, "=== FOC开环控制测试开始 ===\r\n");
    
    /* 1. 电机对齐测试 */
    vofa_print(&huart1, "\r\n[步骤1] 电机对齐测试\r\n");
    test_open_loop_alignment();
    
    /* 等待一段时间 */
    HAL_Delay(2000);
    
    /* 2. 开环低速旋转测试 */
    vofa_print(&huart1, "\r\n[步骤2] 开环低速旋转测试\r\n");
    test_open_loop_constant_speed(0.2f, 100.0f, 5000);  // 20%电压，100RPM，5秒
    
    /* 等待一段时间 */
    HAL_Delay(2000);
    
    /* 3. 开环变速测试 */
    vofa_print(&huart1, "\r\n[步骤3] 开环变速测试\r\n");
    test_open_loop_variable_speed(0.3f, 50.0f, 300.0f, 50.0f, 2000);  // 30%电压，50-300RPM，步进50RPM，每步2秒
    
    /* 等待一段时间 */
    HAL_Delay(2000);
    
    /* 4. 开环正反转测试 */
    vofa_print(&huart1, "\r\n[步骤4] 开环正反转测试\r\n");
    test_open_loop_direction_test(0.25f, 200.0f, 3, 3000);  // 25%电压，200RPM，3次循环，每方向3秒
    
    /* 测试完成，停止电机 */
    tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
    vofa_print(&huart1, "\r\n=== FOC开环控制测试完成 ===\r\n");
}

/**
 * @brief 开环对齐测试
 * 执行电机对齐操作，确定编码器零点偏移
 */
void test_open_loop_alignment(void)
{
    foc_t hfoc;
    pid_controller_t pid_id, pid_iq, pid_speed;
    
    /* 初始化PID控制器 */
    pid_init(&pid_id, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f);
    pid_init(&pid_iq, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f); 
    pid_init(&pid_speed, 1.0f, 0.1f, 0.0f, 5.0f, -5.0f);
    
    /* 初始化FOC控制器 */
    foc_init(&hfoc, &pid_id, &pid_iq, &pid_speed);
    
    /* 执行对齐 */
    foc_alignment(&hfoc);
    
    vofa_print(&huart1, "电机对齐完成，零点偏移: %.4f rad\r\n", hfoc.zero_offset);
}

/**
 * @brief 开环低速旋转测试
 * 以固定低速开环控制电机旋转
 * @param voltage 电压幅值 (0.0-1.0)
 * @param speed_rpm 目标转速 (RPM)
 * @param duration_ms 测试持续时间 (毫秒)
 */
void test_open_loop_constant_speed(float voltage, float speed_rpm, uint32_t duration_ms)
{
    foc_t hfoc;
    pid_controller_t pid_id, pid_iq, pid_speed;
    uint32_t start_time, current_time;
    float angle_mechanical = 0.0f;
    float angle_electrical = 0.0f;
    float speed_rad_per_sec;
    
    /* 初始化PID控制器 */
    pid_init(&pid_id, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f);
    pid_init(&pid_iq, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f); 
    pid_init(&pid_speed, 1.0f, 0.1f, 0.0f, 5.0f, -5.0f);
    
    /* 初始化FOC控制器 */
    foc_init(&hfoc, &pid_id, &pid_iq, &pid_speed);
    
    /* 执行对齐 */
    foc_alignment(&hfoc);
    
    /* 转换转速单位：RPM -> rad/s */
    speed_rad_per_sec = speed_rpm * 2.0f * 3.14159265359f / 60.0f;
    
    vofa_print(&huart1, "开环恒速测试: 电压=%.1f%%, 转速=%.0f RPM, 持续时间=%d ms\r\n", 
                voltage * 100.0f, speed_rpm, duration_ms);
    
    /* 获取开始时间 */
    start_time = HAL_GetTick();
    
    /* 开环控制循环 */
    while (1)
    {
        /* 获取当前时间 */
        current_time = HAL_GetTick();
        
        /* 检查是否超时 */
        if ((current_time - start_time) >= duration_ms)
        {
            break;
        }
        
        /* 更新机械角度 */
        angle_mechanical += speed_rad_per_sec * 0.001f;  // 假设1ms执行一次
        
        /* 保持角度在0-2π范围内 */
        if (angle_mechanical >= 2.0f * 3.14159265359f)
        {
            angle_mechanical -= 2.0f * 3.14159265359f;
        }
        
        /* 计算电角度 */
        angle_electrical = angle_mechanical * hfoc.pole_pairs;
        
        /* 减去零点偏移 */
        angle_electrical -= hfoc.zero_offset;
        
        /* 保持角度在0-2π范围内 */
        if (angle_electrical < 0.0f)
        {
            angle_electrical += 2.0f * 3.14159265359f;
        }
        else if (angle_electrical >= 2.0f * 3.14159265359f)
        {
            angle_electrical -= 2.0f * 3.14159265359f;
        }
        
        /* 开环控制 */
        foc_open_loop(&hfoc, voltage, angle_electrical);
        
        /* 延时1ms */
        HAL_Delay(1);
    }
    
    /* 停止电机 */
    tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
    
    vofa_print(&huart1, "开环恒速测试完成\r\n");
}

/**
 * @brief 开环变速测试
 * 从低速到高速逐渐增加转速
 * @param voltage 电压幅值 (0.0-1.0)
 * @param speed_min_rpm 最小转速 (RPM)
 * @param speed_max_rpm 最大转速 (RPM)
 * @param step_rpm 转速步进 (RPM)
 * @param step_duration_ms 每步持续时间 (毫秒)
 */
void test_open_loop_variable_speed(float voltage, float speed_min_rpm, float speed_max_rpm, float step_rpm, uint32_t step_duration_ms)
{
    foc_t hfoc;
    pid_controller_t pid_id, pid_iq, pid_speed;
    float current_speed_rpm;
    uint32_t start_time, current_time;
    float angle_mechanical = 0.0f;
    float angle_electrical = 0.0f;
    float speed_rad_per_sec;
    
    /* 初始化PID控制器 */
    pid_init(&pid_id, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f);
    pid_init(&pid_iq, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f); 
    pid_init(&pid_speed, 1.0f, 0.1f, 0.0f, 5.0f, -5.0f);
    
    /* 初始化FOC控制器 */
    foc_init(&hfoc, &pid_id, &pid_iq, &pid_speed);
    
    /* 执行对齐 */
    foc_alignment(&hfoc);
    
    vofa_print(&huart1, "开环变速测试: 电压=%.1f%%, 转速范围=%.0f-%.0f RPM, 步进=%.0f RPM, 每步持续时间=%d ms\r\n", 
                voltage * 100.0f, speed_min_rpm, speed_max_rpm, step_rpm, step_duration_ms);
    
    /* 从低速到高速 */
    for (current_speed_rpm = speed_min_rpm; current_speed_rpm <= speed_max_rpm; current_speed_rpm += step_rpm)
    {
        /* 转换转速单位：RPM -> rad/s */
        speed_rad_per_sec = current_speed_rpm * 2.0f * 3.14159265359f / 60.0f;
        
        vofa_print(&huart1, "当前转速: %.0f RPM\r\n", current_speed_rpm);
        
        /* 获取开始时间 */
        start_time = HAL_GetTick();
        
        /* 开环控制循环 */
        while (1)
        {
            /* 获取当前时间 */
            current_time = HAL_GetTick();
            
            /* 检查是否超时 */
            if ((current_time - start_time) >= step_duration_ms)
            {
                break;
            }
            
            /* 更新机械角度 */
            angle_mechanical += speed_rad_per_sec * 0.001f;  // 假设1ms执行一次
            
            /* 保持角度在0-2π范围内 */
            if (angle_mechanical >= 2.0f * 3.14159265359f)
            {
                angle_mechanical -= 2.0f * 3.14159265359f;
            }
            
            /* 计算电角度 */
            angle_electrical = angle_mechanical * hfoc.pole_pairs;
            
            /* 减去零点偏移 */
            angle_electrical -= hfoc.zero_offset;
            
            /* 保持角度在0-2π范围内 */
            if (angle_electrical < 0.0f)
            {
                angle_electrical += 2.0f * 3.14159265359f;
            }
            else if (angle_electrical >= 2.0f * 3.14159265359f)
            {
                angle_electrical -= 2.0f * 3.14159265359f;
            }
            
            /* 开环控制 */
            foc_open_loop(&hfoc, voltage, angle_electrical);
            
            /* 延时1ms */
            HAL_Delay(1);
        }
    }
    
    /* 从高速到低速 */
    for (current_speed_rpm = speed_max_rpm; current_speed_rpm >= speed_min_rpm; current_speed_rpm -= step_rpm)
    {
        /* 转换转速单位：RPM -> rad/s */
        speed_rad_per_sec = current_speed_rpm * 2.0f * 3.14159265359f / 60.0f;
        
        vofa_print(&huart1, "当前转速: %.0f RPM\r\n", current_speed_rpm);
        
        /* 获取开始时间 */
        start_time = HAL_GetTick();
        
        /* 开环控制循环 */
        while (1)
        {
            /* 获取当前时间 */
            current_time = HAL_GetTick();
            
            /* 检查是否超时 */
            if ((current_time - start_time) >= step_duration_ms)
            {
                break;
            }
            
            /* 更新机械角度 */
            angle_mechanical += speed_rad_per_sec * 0.001f;  // 假设1ms执行一次
            
            /* 保持角度在0-2π范围内 */
            if (angle_mechanical >= 2.0f * 3.14159265359f)
            {
                angle_mechanical -= 2.0f * 3.14159265359f;
            }
            
            /* 计算电角度 */
            angle_electrical = angle_mechanical * hfoc.pole_pairs;
            
            /* 减去零点偏移 */
            angle_electrical -= hfoc.zero_offset;
            
            /* 保持角度在0-2π范围内 */
            if (angle_electrical < 0.0f)
            {
                angle_electrical += 2.0f * 3.14159265359f;
            }
            else if (angle_electrical >= 2.0f * 3.14159265359f)
            {
                angle_electrical -= 2.0f * 3.14159265359f;
            }
            
            /* 开环控制 */
            foc_open_loop(&hfoc, voltage, angle_electrical);
            
            /* 延时1ms */
            HAL_Delay(1);
        }
    }
    
    /* 停止电机 */
    tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
    
    vofa_print(&huart1, "开环变速测试完成\r\n");
}

/**
 * @brief 开环正反转测试
 * 测试电机正转和反转
 * @param voltage 电压幅值 (0.0-1.0)
 * @param speed_rpm 目标转速 (RPM)
 * @param cycles 正反转循环次数
 * @param duration_ms 每个方向持续时间 (毫秒)
 */
void test_open_loop_direction_test(float voltage, float speed_rpm, uint8_t cycles, uint32_t duration_ms)
{
    foc_t hfoc;
    pid_controller_t pid_id, pid_iq, pid_speed;
    uint8_t i;
    uint32_t start_time, current_time;
    float angle_mechanical = 0.0f;
    float angle_electrical = 0.0f;
    float speed_rad_per_sec;
    
    /* 初始化PID控制器 */
    pid_init(&pid_id, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f);
    pid_init(&pid_iq, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f); 
    pid_init(&pid_speed, 1.0f, 0.1f, 0.0f, 5.0f, -5.0f);
    
    /* 初始化FOC控制器 */
    foc_init(&hfoc, &pid_id, &pid_iq, &pid_speed);
    
    /* 执行对齐 */
    foc_alignment(&hfoc);
    
    /* 转换转速单位：RPM -> rad/s */
    speed_rad_per_sec = speed_rpm * 2.0f * 3.14159265359f / 60.0f;
    
    vofa_print(&huart1, "开环正反转测试: 电压=%.1f%%, 转速=%.0f RPM, 循环次数=%d, 每方向持续时间=%d ms\r\n", 
                voltage * 100.0f, speed_rpm, cycles, duration_ms);
    
    /* 正反转循环 */
    for (i = 0; i < cycles; i++)
    {
        /* 正转 */
        vofa_print(&huart1, "第%d次循环: 正转\r\n", i + 1);
        start_time = HAL_GetTick();
        
        while (1)
        {
            /* 获取当前时间 */
            current_time = HAL_GetTick();
            
            /* 检查是否超时 */
            if ((current_time - start_time) >= duration_ms)
            {
                break;
            }
            
            /* 更新机械角度 */
            angle_mechanical += speed_rad_per_sec * 0.001f;  // 假设1ms执行一次
            
            /* 保持角度在0-2π范围内 */
            if (angle_mechanical >= 2.0f * 3.14159265359f)
            {
                angle_mechanical -= 2.0f * 3.14159265359f;
            }
            
            /* 计算电角度 */
            angle_electrical = angle_mechanical * hfoc.pole_pairs;
            
            /* 减去零点偏移 */
            angle_electrical -= hfoc.zero_offset;
            
            /* 保持角度在0-2π范围内 */
            if (angle_electrical < 0.0f)
            {
                angle_electrical += 2.0f * 3.14159265359f;
            }
            else if (angle_electrical >= 2.0f * 3.14159265359f)
            {
                angle_electrical -= 2.0f * 3.14159265359f;
            }
            
            /* 开环控制 */
            foc_open_loop(&hfoc, voltage, angle_electrical);
            
            /* 延时1ms */
            HAL_Delay(1);
        }
        
        /* 停止电机 */
        tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
        HAL_Delay(500);  // 停止500ms
        
        /* 反转 */
        vofa_print(&huart1, "第%d次循环: 反转\r\n", i + 1);
        start_time = HAL_GetTick();
        
        while (1)
        {
            /* 获取当前时间 */
            current_time = HAL_GetTick();
            
            /* 检查是否超时 */
            if ((current_time - start_time) >= duration_ms)
            {
                break;
            }
            
            /* 更新机械角度 */
            angle_mechanical -= speed_rad_per_sec * 0.001f;  // 反转，负方向
            
            /* 保持角度在0-2π范围内 */
            if (angle_mechanical < 0.0f)
            {
                angle_mechanical += 2.0f * 3.14159265359f;
            }
            
            /* 计算电角度 */
            angle_electrical = angle_mechanical * hfoc.pole_pairs;
            
            /* 减去零点偏移 */
            angle_electrical -= hfoc.zero_offset;
            
            /* 保持角度在0-2π范围内 */
            if (angle_electrical < 0.0f)
            {
                angle_electrical += 2.0f * 3.14159265359f;
            }
            else if (angle_electrical >= 2.0f * 3.14159265359f)
            {
                angle_electrical -= 2.0f * 3.14159265359f;
            }
            
            /* 开环控制 */
            foc_open_loop(&hfoc, voltage, angle_electrical);
            
            /* 延时1ms */
            HAL_Delay(1);
        }
        
        /* 停止电机 */
        tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
        HAL_Delay(500);  // 停止500ms
    }
    
    vofa_print(&huart1, "开环正反转测试完成\r\n");
}