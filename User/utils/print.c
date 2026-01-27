#include "print.h"
#include "bsp/usart.h"
#include <string.h>

/* JustFloat 协议帧尾: 0x00, 0x00, 0x80, 0x7f */
static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7f};

/**
 * @brief 发送 JustFloat 协议数据
 * @param data float数组指针(一次存num个元素)
 * @param num float数据个数(通道数)
 */
void printf_vofa(float *data, uint16_t num)
{
    /* 定义发送缓冲区 */
    static uint8_t tx_buffer[128];
    uint16_t data_len = num * sizeof(float);

    /* 复制数据部分 */
    memcpy(tx_buffer, data, data_len);

    /* 复制帧尾 */
    memcpy(tx_buffer + data_len, justfloat_tail, 4);

    /* 一次性发送数据和帧尾，避免被其他串口发送打断 */
    usart1_send_data(tx_buffer, data_len + 4);
}
