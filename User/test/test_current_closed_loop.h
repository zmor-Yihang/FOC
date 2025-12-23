#ifndef __TEST_CURRENT_CLOSED_LOOP_H__
#define __TEST_CURRENT_CLOSED_LOOP_H__

#include "./foc/foc.h"
#include "./bsp/as5047.h"
#include "./bsp/adc.h"
#include "./foc/clark_park.h"
#include "./utils/delay.h"
#include "./bsp/key.h"
#include "./bsp/tim.h"

void test_current_closed_loop(void);

#endif /* __TEST_CURRENT_CLOSED_LOOP_H__ */