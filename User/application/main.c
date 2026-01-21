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

    // speed_closed_with_smo_init(2000);

    sensorless_smo_init(3000);

    while (1)
    {
        if (key_scan() == 1)
        {
            printf("Stop!\n");
            break;
        }
        // print_speed_smo_info();
        print_sensorless_info();
    }
}
