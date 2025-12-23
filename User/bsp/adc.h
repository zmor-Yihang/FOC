#ifndef __ADC_H__
#define __ADC_H__

#include "stm32g4xx_hal.h"
#include "config/type_config.h"
#include "test/test_current_closed_loop.h"

/* ADC句柄声明 */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

#define ADC_REF_VOLTAGE 1.65f              /* ADC参考电压(理论零点)，单位V */
#define ADC_CURRENT_SCALE (100.0f / 16.5f) /* 电流传感器比例系数，单位V/A */
#define ADC_UDC_SCALE 25.0f /* Udc母线电压转换比例，单位V/bit */

/* 函数声明 */
extern volatile uint8_t adc_injected_cplt_flag;

void adc1_init(void);

void adc1_get_offset(adc_offset_t *offsets); /* 调试接口，仅供测试使用 */

void adc1_get_regular_values(adc_values_t *values);
void adc1_get_injected_values(adc_values_t *values);

#endif /* __ADC_H__ */