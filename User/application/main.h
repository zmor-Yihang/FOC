#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32g4xx_hal.h"

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

#include "utils/fast_sin_cos.h"
#include "utils/vofa.h"

#include "test/test_clark_park.h"
#include "test/test_led.h"
#include "test/test_key.h"
#include "test/test_adc.h"
#include "test/test_tim1.h"
#include "test/test_svpwm.h"
#include "test/test_tim1.h"
#include "test/test_pid.h"

#include "test/test_as5047.h"
#include "test/test_open_loop.h"
#include "test/test_rotation_simulation.h"



#endif /* __MAIN_H__ */
