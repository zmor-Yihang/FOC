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


// int main()
// {
//     motor_ctrl_bsp_init();
//     motor_ctrl_init();
//     test_speed_loop_init(1000.0f); // 启动，目标1000 RPM

//     while (1)
//     {
//         test_speed_loop_print_status(); // 每100ms打印对比数据
//         HAL_Delay(10);
//     }
// }
