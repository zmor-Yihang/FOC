#include "main.h"
#include <string.h>
#include <stdio.h>

// 发送缓冲区
uint8_t txBuffer[64] = {0};
// 回传缓冲区
uint8_t echoBuffer[64] = {0};

int main(void)
{
    HAL_Init();
    clock_init();
    usart1_init();
    
    printf("USART1 DMA Test Start...\r\n");
    
    // 发送测试数据
    strcpy((char*)txBuffer, "Hello from DMA!\r\n");
    usart1_sendData(txBuffer, strlen((char*)txBuffer));
    
    while (1)
    {
            printf("Loop iteration, rxCompleteFlag = %d, rxSize = %d\r\n", rxCompleteFlag, rxSize);
        // 检查是否接收到数据
        if(rxCompleteFlag)
        {
            rxCompleteFlag = 0;  // 清除接收完成标志
            
            // 将接收到的数据复制到回传缓冲区
            if(rxSize <= 64) {
                memcpy(echoBuffer, (uint8_t*)rxBuffer, rxSize);
                
                // 先回传接收到的数据
                usart1_sendData(echoBuffer, rxSize);
                
                // 延时一小段时间确保数据发送完成
                HAL_Delay(10);
                
                // 再发送确认信息
                printf("\nReceived %d bytes\r\n", rxSize);
            }
        }
        printf("Waiting for data...\r\n");
        HAL_Delay(2000);  // 简单延时
    }
}