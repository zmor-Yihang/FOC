#include "test_foc_open.h"

void test_foc_open(void)
{

    dq_t u_dq = {
        .d = 0.0f,
        .q = 2.0f,
    };

    float angle_el = 0.0f;

    adc_values_t injected_values;
    while (1)
    {
        angle_el += 0.005;

        foc_open_loop(u_dq, angle_el);

        adc1_get_injected_values(&injected_values);

        delay_us(10);

        // printf("loop is running\n");

        if (key_scan() == 1)
        {
            tim1_set_pwm_duty(0, 0, 0);
            break;
        }
    }
    printf("motor is stop!\r\n");
}