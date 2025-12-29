#ifndef __TEST_FOC_OPEN_H__
#define __TEST_FOC_OPEN_H__

#include "stm32g4xx_hal.h"
#include "config/type_config.h"

#include <string.h>
#include <stdio.h>

#include "bsp/clock.h"
#include "bsp/usart.h"
#include "bsp/key.h"
#include "bsp/led.h"
#include "bsp/adc.h"

#include "foc/clark_park.h"
#include "foc/pid.h"
#include "foc/svpwm.h"
#include "foc/foc.h"
#include "foc/foc.h"

#include "utils/fast_sin_cos.h"
#include "utils/print.h"
#include "utils/delay.h"

void test_foc_open(void);

#endif /* __TEST_FOC_OPEN_H__ */
