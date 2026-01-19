#ifndef __SPEED_CLOSED_H__
#define __SPEED_CLOSED_H__

#include "foc/foc.h"
#include "bsp/as5047.h"
#include "utils/print.h"

void speed_closed_init(float speed_rpm);
void print_speed_info(void);

#endif /* __SPEED_CLOSED_H__ */
