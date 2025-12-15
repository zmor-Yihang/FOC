#include "vofa.h"

/*
 * 发送数据到 VOFA+ 上位机
 * huart: 你的串口句柄，比如 &huart1 或 &huart2
 * fmt:   格式化字符串，例如 "%f,%f\n"
 * ...:   对应的变量
 */
void vofa_print(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[128]; // 定义发送缓冲区，128字节通常够用了
    va_list args;

    // 1. 开始处理可变参数
    va_start(args, fmt);

    // 2. 将格式化数据打印到 buffer 中
    // vsnprintf 比 sprintf 安全，防止缓冲区溢出
    vsnprintf(buffer, sizeof(buffer), fmt, args);

    // 3. 结束处理可变参数
    va_end(args);

    // 4. 通过串口发送出去
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), 100);
}
