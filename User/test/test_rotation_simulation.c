#include "test_rotation_simulation.h"

/**
 * @brief 测试电角度自增模拟旋转功能
 */
void test_rotation_simulation(void)
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
    
    vofa_print(&huart1, "=== 电角度自增模拟旋转测试 ===\r\n");
    
    /* 测试1: 低速旋转 */
    vofa_print(&huart1, "测试1: 低速旋转 (100RPM, 30%%电压, 持续5秒)\r\n");
    foc_simulate_rotation(&hfoc, 0.1f, 100.0f, 5000);
    HAL_Delay(2000);
    
    /* 测试2: 中速旋转 */
    vofa_print(&huart1, "测试2: 中速旋转 (300RPM, 50%%电压, 持续5秒)\r\n");
    foc_simulate_rotation(&hfoc, 0.2f, 300.0f, 5000);
    HAL_Delay(2000);
    
    /* 测试3: 高速旋转 */
    vofa_print(&huart1, "测试3: 高速旋转 (600RPM, 70%%电压, 持续5秒)\r\n");
    foc_simulate_rotation(&hfoc, 0.3f, 600.0f, 5000);
    HAL_Delay(2000);
    
    /* 测试完成，停止电机 */
    tim1_set_pwm_duty(0.0f, 0.0f, 0.0f);
    vofa_print(&huart1, "=== 电角度自增模拟旋转测试完成 ===\r\n");
}