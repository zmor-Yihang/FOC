#include "main.h"

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
    foc_alignment(); /* 电机零点对齐 */

    while (1)
    {

        if (key_scan() == 1)
        {
            printf("Loop is exit!\n");
            break;
        }
        HAL_Delay(100);
    }
}