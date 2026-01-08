#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "bsp/clock.h"
#include "bsp/usart.h"
#include "bsp/key.h"
#include "bsp/led.h"
#include "bsp/adc.h"
#include "bsp/tim.h"
#include "bsp/as5047.h"
#include "foc/clark_park.h"
#include "foc/svpwm.h"
#include "utils/print.h"
#include <stdio.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include "foc/foc.h"
#include "foc/pid.h"

/* 控制模式枚举 */
typedef enum {
    CTRL_MODE_IDLE = 0,      /* 空闲模式 */
    CTRL_MODE_OPEN_LOOP,     /* 开环模式 */
    CTRL_MODE_CURRENT,       /* 电流闭环模式 */
    CTRL_MODE_SPEED          /* 速度闭环模式 */
} ctrl_mode_t;

/* 初始化电机控制 */
void motor_ctrl_init(void);

/* 系统硬件初始化 */
void motor_ctrl_bsp_init(void);

/* 切换到下一个控制模式 */
void motor_ctrl_switch_mode(void);

/* 停止电机 */
void motor_ctrl_stop(void);

/* 打印状态信息 */
void motor_ctrl_print_status(void);

#endif /* __MOTOR_CTRL_H__ */
