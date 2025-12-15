#ifndef __USART1_H__
#define __USART1_H__

#include "stm32g4xx_hal.h"
#include "stdio.h"
#include "./utils/fifofast.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx; /* 声明USART1接收DMA句柄 */
extern DMA_HandleTypeDef hdma_usart1_tx; /* 声明USART1发送DMA句柄 */

#define RX_BUF_TEMP_SIZE 64 /* 定义临时接收缓冲区大小为64字节 */
#define RX_BUFFER_SIZE 128  /* 定义接收缓冲区大小为128字节 */

void usart1_init(void);
void usart1_send_data(uint8_t *data, uint16_t size);
uint16_t usart1_read_data(uint8_t *buf, uint16_t size);
uint16_t usart1_get_available_buffer(void);
uint8_t usart1_fifo_is_empty(void);

#endif /* __USART1_H__ */
