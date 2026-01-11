#include "main.h"

int main(void)
{
    motor_ctrl_bsp_init();
    motor_ctrl_init();

    printf("FOC Ready! KEY to switch mode\n");

    while (1)
    {
        if (key_scan() == 1)
        {
            motor_ctrl_switch_mode();
        }

        motor_ctrl_print_status();

        HAL_Delay(10);
    }
}
