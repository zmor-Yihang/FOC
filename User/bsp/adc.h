#ifndef __ADC_H__
#define __ADC_H__

#include "stm32g4xx_hal.h"

/* ADC句柄声明 */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/* ADC DMA缓冲区 */
#define ADC_CHANNEL_NUM 4                   /* ADC通道数量 */
extern uint16_t adc_dma_buf[ADC_CHANNEL_NUM]; /* ADC DMA数据缓冲区 */

/* 注入组数据缓冲区 */
extern volatile uint16_t adc_injected_buf[ADC_CHANNEL_NUM]; /* ADC注入组数据缓冲区 */

/* 函数声明 */
void adc1_init(void);
void adc1_start_regular_dma(void);
void adc1_start_injected(void);

#endif /* __ADC_H__ */