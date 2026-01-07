#include "test_foc_open.h"

void test_foc_open(void)
{
    dq_t u_dq = {
        .d = 0.0f,
        .q = 2.0f,
    };

    float angle_el = 0.0f;
    
    float theta_increment = 2.0f * MOTOR_POLE_PAIR * (M_PI / 180.0f);  /* 14度转弧度 */

    adc_values_t injected_values;
    while (1)
    {
        angle_el += theta_increment;

        foc_open_loop(u_dq, angle_el);

        adc1_get_injected_values(&injected_values);

        HAL_Delay(10);  /* 10ms 周期 */

        if (key_scan() == 1)
        {
            tim1_set_pwm_duty(0.5f, 0.5f, 0.5f);
            break;
        }
    }
    printf("motor is stop!\r\n");
}