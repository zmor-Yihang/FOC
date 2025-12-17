#include "test_clark_park.h"

void test_clark_park(void)
{
    float t = 0.01f;
    float theta = 0;

    phase3_t uvw_voltages = {0};
    phase2_t alpha_beta_voltages = {0};
    phase2_t dq_voltages = {0};

    while (1)
    {
        uvw_voltages.axis_1 = 20.0f*fast_cos(2 * 3.14 * 50 * t + 3.1415926/3.0f);
        uvw_voltages.axis_2 = 20.0f*fast_cos(2 * 3.14 * 50 * t - 2.09 + 3.1415926/3.0f);
        uvw_voltages.axis_3 = 20.0f*fast_cos(2 * 3.14 * 50 * t + 2.09 + 3.1415926/3.0f);

        clark_transform(uvw_voltages, &alpha_beta_voltages);
        park_transform(alpha_beta_voltages, theta, &dq_voltages);

        vofa_print(&huart1, "%.3f, %.3f, %.3f, %.3f\r\n", alpha_beta_voltages.axis_1, alpha_beta_voltages.axis_2, dq_voltages.axis_1, dq_voltages.axis_2);
        
        HAL_Delay(100);
        t += 0.0001f;
        theta = 2*3.1415926*50*t;
    }
}