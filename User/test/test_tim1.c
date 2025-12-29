#include "test_tim1.h"


/**
 * @brief TIM1 PWM发波测试函数
 * 测试功能：
 * 1. 阶段1：固定占空比测试（验证PWM能否正常输出）
 * 2. 阶段2：占空比渐变测试（0% -> 100% -> 0%）
 * 3. 阶段3：三相不同占空比测试（验证三通道独立控制）
 */
void test_tim1(void)
{
    float duty = 0.0f;
    uint8_t direction = 1;  /* 1:上升 0:下降 */
    
    /* 初始化TIM1 */
    tim1_init();
    
    printf("=== TIM1 PWM测试开始 ===\r\n");
    
    /* 阶段1：固定占空比测试 */
    printf("[阶段1] 固定占空比50%%测试 (3秒)\r\n");
    tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
    HAL_Delay(3000);
    
    printf("[阶段1] 固定占空比25%%测试 (3秒)\r\n");
    tim1_set_pwm_duty(0.25f, 0.25f, 0.25f);
    HAL_Delay(3000);
    
    printf("[阶段1] 固定占空比75%%测试 (3秒)\r\n");
    tim1_set_pwm_duty(0.75f, 0.75f, 0.75f);
    HAL_Delay(3000);
    
    /* 阶段2：占空比渐变测试 */
    printf("[阶段2] 占空比渐变测试 (0%% -> 100%% -> 0%%)\r\n");
    duty = 0.0f;
    direction = 1;
    
    while (1)
    {
        /* 设置三相相同占空比 */
        tim1_set_pwm_duty(duty, duty, duty);
        
        /* 打印当前占空比 */
        printf("Duty: %.2f%%\r\n", duty * 100.0f);
        
        /* 占空比渐变 */
        if (direction)
        {
            duty += 0.05f;  /* 每次增加5% */
            if (duty >= 1.0f)
            {
                duty = 1.0f;
                direction = 0;  /* 改变方向 */
                printf(">>> 达到100%%, 开始下降\r\n");
            }
        }
        else
        {
            duty -= 0.05f;  /* 每次减少5% */
            if (duty <= 0.0f)
            {
                duty = 0.0f;
                direction = 1;  /* 改变方向 */
                printf(">>> 达到0%%, 开始上升\r\n");
                
                /* 完成一个周期后，进入阶段3 */
                HAL_Delay(1000);
                break;
            }
        }
        
        HAL_Delay(200);  /* 200ms更新一次 */
    }
    
    /* 阶段3：三相不同占空比测试 */
    printf("[阶段3] 三相独立占空比测试\r\n");
    while (1)
    {
        printf("Phase: 25%%, 50%%, 75%%\r\n");
        tim1_set_pwm_duty(0.25f, 0.5f, 0.75f);
        HAL_Delay(2000);
        
        printf("Phase: 75%%, 25%%, 50%%\r\n");
        tim1_set_pwm_duty(0.75f, 0.25f, 0.5f);
        HAL_Delay(2000);
        
        printf("Phase: 50%%, 75%%, 25%%\r\n");
        tim1_set_pwm_duty(0.5f, 0.75f, 0.25f);
        HAL_Delay(2000);
    }
}
