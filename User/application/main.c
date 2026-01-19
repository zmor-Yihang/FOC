#include "main.h"

// int main(void)
// {
//     motor_ctrl_bsp_init();
//     motor_ctrl_init();

//     printf("FOC Ready! KEY to switch mode\n");

//     while (1)
//     {
//         if (key_scan() == 1)
//         {
//             motor_ctrl_switch_mode();
//         }
//         motor_ctrl_print_status();
//         HAL_Delay(10);
//     }
// }

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

    // current_closed_init(0.0f, 0.2f);
    speed_closed_init(400);
    while (1)
    {
        if (key_scan() == 1)
        {
            printf("Stop!\n");
            break;
        }
        print_speed_info();
        HAL_Delay(10);
    }
}
