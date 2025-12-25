#include "test_adc.h"
#include "bsp/adc.h"
#include "bsp/usart.h"

void test_adc(void)
{
    adc_values_t regular_values;
    adc_values_t injected_values;

    while (1)
    {
        adc1_get_regular_values(&regular_values);
        adc1_get_injected_values(&injected_values);

        printf("%f, %f, %f, %f, %f, %f, %f, %f\n", 
               regular_values.ia, regular_values.ib, regular_values.ic, regular_values.udc,
               injected_values.ia, injected_values.ib, injected_values.ic, injected_values.udc);

        HAL_Delay(2000);
    }
}
