#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart1_init();

    led1_init(); // 初始化LED1
    led2_init(); // 初始化LED2

    key_init(); // 初始化按键

    as5047_init(); // 初始化AS5047P编码器

    adc1_init(); // 初始化ADC1

    tim3_init(); // 初始化TIM3用于编码器接口
    tim1_init(); // 初始化TIM1用于PWM输出

    led2_on(); // 点亮LED2使能
    foc_t hfoc;
    pid_controller_t pid_id, pid_iq, pid_speed;
    pid_init(&pid_id, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f);
    pid_init(&pid_iq, 0.1f, 0.01f, 0.0f, 5.0f, -5.0f);
    pid_init(&pid_speed, 1.0f, 0.1f, 0.0f, 5.0f, -5.0f);
    foc_init(&hfoc, &pid_id, &pid_iq, &pid_speed);
-
    adc_values_t adc_vals = {0};
    adc1_calibrate_zero(&adc_vals); // 校准ADC1零点

    // 运行开环测试
    // test_open_loop();
    foc_alignment(&hfoc);


    test_rotation_simulation();
    while (1)
    {

    }
}
------------------------------