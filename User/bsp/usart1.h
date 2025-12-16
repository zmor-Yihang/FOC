#ifndef __USART1_H__
#define __USART1_H__

#include "stm32g4xx_hal.h"
#include "stdio.h"
#include "./utils/fifofast.h"

extern UART_HandleTypeDef huart1; 
extern DMA_HandleTypeDef hdma_usart1_rx; /* 声明USART1接收DMA句柄 */
extern DMA_HandleTypeDef hdma_usart1_tx; /* 声明USART1发送DMA句柄 */

#define RX_BUFFER_SIZE 128           /* 定义接收缓冲区大小为128字节 */
extern uint8_t rxBuffer[];              /* 接收缓冲区 */
extern volatile uint16_t rxSize;        /* 实际接收到的数据长度 */
extern volatile uint8_t rxCompleteFlag; /* 接收完成标志 */

void usart1_init(void);
void usart1_sendData(uint8_t *data, uint16_t size);
void usart1_receiveData(uint8_t *data, uint16_t size);

#endif /* __USART1_H__ */

