#include "main.h"

int main(void)
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

    sensorless_luenberger_init(2000); // Luenberger 无感
    // speed_closed_with_luenberger_init(200); //  Luenberger 速度闭环
    // sensorless_smo_init(1000); // 滑模无感
    while (1)
    {
        if (key_scan() == 1)
        {
            printf("Stop!\n");
            break;
        }

        print_sensorless_luenberger_info();
        // print_speed_luenberger_info();
        // print_sensorless_smo_info();
    }
}
