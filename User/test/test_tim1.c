#include "test_tim1.h"


/**
 * @brief TIM1 PWM发波测试函数
 * 测试功能：
 * 1. TIM1三相PWM输出（PA8-PA10和PB13-PB15互补）
 * 2. 动态改变占空比，实现三相SVPWM波形
 * 3. 定期打印占空比到UART1
 */
void test_tim1(void)
{
    float t = 0.0f;
    float duty1, duty2, duty3;
    float frequency = 50.0f;  /* 50Hz基频 */
    float amplitude = 0.5f;   /* 幅值50% */
    uint32_t test_count = 0;
    
    /* 初始化TIM1 */
    tim1_init();
    
    
    while (1)
    {
        /* 生成三相SVPWM波形 */
        duty1 = 0.5f + amplitude * fast_sin(2 * 3.14159265f * frequency * t);
        duty2 = 0.5f + amplitude * fast_sin(2 * 3.14159265f * frequency * t - 2.0944f); /* -120° */
        duty3 = 0.5f + amplitude * fast_sin(2 * 3.14159265f * frequency * t + 2.0944f); /* +120° */
        
        /* 设置TIM1三相占空比 */
        tim1_set_pwm_duty(duty1, duty2, duty3);
        
        /* 每100ms打印一次占空比 */
        if (test_count % 10 == 0)
        {
            vofa_print(&huart1, "Duty: %.3f, %.3f, %.3f\r\n",
                       duty1, duty2, duty3);
        }
        
        HAL_Delay(10); /* 10ms更新一次，总周期1000ms（50Hz*20） */
        t += 0.01f / 1000.0f; /* 增加时间步长 */
        test_count++;
    }
}
