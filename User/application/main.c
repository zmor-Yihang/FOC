#include "main.h"


static foc_t foc_handle;

int main(void)
{
    HAL_Init();
    clock_init();
    usart1_init();

    led1_init(); /* 初始化LED1 */
    key_init();  /* 初始化按键 */

    led2_init(); /* 初始化LED2，使能PWM输出 */

    as5047_init(); /* 初始化AS5047P编码器 */

    tim3_init(); /* 初始化TIM3用于编码器接口 */
    tim1_init(); /* 初始化TIM1用于PWM输出 */

    adc1_init();     /* 初始化ADC1 */
    test_svpwm(); /* 测试SVPWM算法 */

    foc_alignment(&foc_handle); /* 电机零点对齐 */
    // test_raw_six_step();

    test_foc_open(); /* 测试开环控制 */

    // test_current_closed_loop(); /* 测试电流闭环控制 */

    test_speed_closed_loop(); /* 测试速度闭环控制 */

    while (1)
    {


        if (key_scan() == 1)
        {
            printf("Loop is exit!\n");
            break;
        }
        HAL_Delay(10);
    }
}