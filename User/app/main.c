#include "main.h"


int main(void)
{
    HAL_Init();
    clock_init();
    usart1_init();

    float t = 0.0f;
    int count = 0;
    uint32_t lastTick = 0;  // 用于非阻塞定时
    
    while (1)
    {
        /* 检查是否接收到数据 - 每次循环都检查 */
        if (rxCompleteFlag)
        {
            rxCompleteFlag = 0;  // 清除标志
            // 处理接收到的数据，rxBuffer中有rxSize字节数据
            // 示例：回显接收到的数据
            usart1_sendData(rxBuffer, rxSize);
        }

        /* 非阻塞定时：每50ms执行一次发送 */
        if (HAL_GetTick() - lastTick >= 50)
        {
            lastTick = HAL_GetTick();
            
            t += 0.1f;
            count++;
            float ch1 = fast_sin(t);           // 正弦波
            float ch2 = fast_cos(t);           // 余弦波
            float ch3 = fast_sin(t) * fast_cos(t); // 或者是其他的变量，比如 PID 的 Error

            // 调用刚才写的函数
            // 注意：格式必须是 "数字,数字,数字\n"
            // %.2f 表示保留2位小数，既节省带宽又足够清晰
            vofa_print(&huart1, "zmor:%.2f,%.2f,%.2f\r\n", ch1, ch2, ch3);

            printf("test %d: %.2f,%.2f,%.2f\r\n", count, ch1, ch2, ch3);
            // 如果想加标签(比如波形名字叫 motor)，可以这样写：
            // vofa_print(&huart1, "motor:%.2f,%.2f,%.2f\n", ch1, ch2, ch3);
        }
    }
}
