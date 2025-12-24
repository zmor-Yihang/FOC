#ifndef __AS5047_H__
#define __AS5047_H__

#include "stm32g4xx_hal.h"
#include "spi.h"
#include <math.h>

/* AS5047P 寄存器地址定义 */
#define AS5047_REG_NOP          0x0000  /* 空操作 */
#define AS5047_REG_ERRFL        0x0001  /* 错误标志寄存器 */
#define AS5047_REG_PROG         0x0003  /* 编程寄存器 */
#define AS5047_REG_DIAAGC       0x3FFC  /* 诊断和AGC */
#define AS5047_REG_MAG          0x3FFD  /* CORDIC 幅度 */
#define AS5047_REG_ANGLEUNC     0x3FFE  /* 无补偿角度值 */
#define AS5047_REG_ANGLECOM     0x3FFF  /* 带补偿角度值 (DAE) */

/* 非易失性寄存器 (Non-Volatile) */
#define AS5047_REG_ZPOSM        0x0016  /* 零位高8位 */
#define AS5047_REG_ZPOSL        0x0017  /* 零位低6位 + 补偿使能 */
#define AS5047_REG_SETTINGS1    0x0018  /* 设置寄存器1 */
#define AS5047_REG_SETTINGS2    0x0019  /* 设置寄存器2 */

/* AS5047P 分辨率 */
#define AS5047_RESOLUTION       16384   /* 14位分辨率 (2^14) */

/* 速度计算参数 */
#define AS5047_SPEED_SAMPLE_TIME 0.001f /* 采样周期 (秒) - 1ms */
#define AS5047_SPEED_FILTER_ALPHA 0.2f  /* 速度滤波系数 (一阶低通) */

void as5047_init(void);
float as5047_get_angle_rad(void);
float as5047_get_speed_rpm(void);
uint16_t as5047_get_error(void);




#endif /* __AS5047_H__ */
