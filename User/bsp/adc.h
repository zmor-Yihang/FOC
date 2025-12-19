#ifndef __ADC_H__
#define __ADC_H__

#include "stm32g4xx_hal.h"
#include "config/type_config.h"

/* ADC句柄声明 */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

#define ADC_REF_VOLTAGE 1.65f /* ADC参考电压(理论零点)，单位V */

#define ADC_CURRENT_SCALE (100.0f / 16.5f) /* 电流传感器比例系数，单位V/A */
#define ADC_UDC_SCALE 25.0f                /* Udc母线电压转换比例，单位V/bit */

/* ADC 缓冲区 */
extern uint16_t adc_regular_buf[4];           /* ADC DMA数据缓冲区(规则组) */
extern volatile uint16_t adc_injected_buf[4]; /* ADC注入组数据缓冲区 */

/* 函数声明 */
void adc1_init(void);
void adc1_start_regular_dma(void);
void adc1_stop_injected(void);

void adc1_calibrate_zero(adc_values_t *values);
void adc1_value_convert(uint16_t *adc_buf, adc_values_t *values);

#endif /* __ADC_H__ */