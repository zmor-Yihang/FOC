#include "test_led.h"

void test_led1(void)
{
    led1_toggle();
    HAL_Delay(1000);
}
