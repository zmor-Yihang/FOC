#include "test_clark_park.h"

void test_clark_park(void)
{
    float t = 0.01f;
    float theta = 0;

    abc_t uvw_voltages = {0};
    alphabeta_t alpha_beta_voltages = {0};
    dq_t dq_voltages = {0};

    while (1)
    {
        uvw_voltages.a = 20.0f*fast_cos(2 * 3.14 * 50 * t + 3.1415926/3.0f);
        uvw_voltages.b = 20.0f*fast_cos(2 * 3.14 * 50 * t - 2.09 + 3.1415926/3.0f);
        uvw_voltages.c = 20.0f*fast_cos(2 * 3.14 * 50 * t + 2.09 + 3.1415926/3.0f);

        alpha_beta_voltages = clark_transform(uvw_voltages);
        dq_voltages = park_transform(alpha_beta_voltages, theta);

        vofa_print(&huart1, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",uvw_voltages.a, uvw_voltages.b, uvw_voltages.c,
             alpha_beta_voltages.alpha, alpha_beta_voltages.beta, dq_voltages.d, dq_voltages.q);
        
        HAL_Delay(100);
        t += 0.0001f;
        theta = 2*3.1415926*50*t;
    }
}