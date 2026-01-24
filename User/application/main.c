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

    sensorless_smo_init(800);
    // speed_closed_init(6500);
    // flux_weak_speed_closed_init(6500);

    while (1)
    {
        if (key_scan() == 1)
        {
            printf("Stop!\n");
            break;
        }
        print_sensorless_info();
        // print_speed_info();
        // print_flux_weak_speed_info();
    }
}
