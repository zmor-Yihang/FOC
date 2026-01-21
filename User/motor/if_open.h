#ifndef __IF_OPEN_H__
#define __IF_OPEN_H__

#include "foc/foc.h"
#include "utils/ramp.h"
#include "utils/print.h"

void if_open_init(float speed_rpm, float iq);
void print_if_current_info(void);

#endif /*__IF_OPEN_H__*/
