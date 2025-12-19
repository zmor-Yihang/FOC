#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart1_init();
    led1_init(); // 初始化LED1
    key_init();  // 初始化按键
    tim1_init(); // 初始化TIM1用于PWM输出


    while (1)
    {
        // test_clark_park();
        // test_adc(); // 取消注释测试ADC
        // test_tim1(); // 取消注释测试TIM1
        // test_svpwm(); // 取消注释测试SVPWM
        // test_tim3_encoder(); // 取消注释测试TIM3编码器
        // test_foc_triple_loop_vofa(&huart1);
        // test_encoder();
        test_as5047();
        // test_foc_full_closed_loop();
    }
}
