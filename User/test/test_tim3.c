#include "test_tim3.h"

/* 编码器位置跟踪变量 */
static int32_t encoder_position = 0;
static uint16_t last_count = 0;
static uint32_t ic3_capture_value = 0;
static uint8_t ic3_capture_flag = 0;

/**
 * @brief TIM3 编码器测试函数
 * 测试功能：
 * 1. 读取编码器AB相计数（通道1、2）
 * 2. 计算编码器位置和方向
 * 3. 读取编码器Z相脉冲（通道3输入捕获）
 * 4. 定期打印编码器数据到UART
 */
void test_tim3_encoder(void)
{
    uint16_t current_count = 0;
    int16_t delta = 0;
    uint32_t test_count = 0;
    
    /* 初始化TIM3 */
    tim3_init();
    tim3_start();
    
    /* 获取初始计数值 */
    last_count = __HAL_TIM_GET_COUNTER(&htim3);
    
    vofa_print(&huart1, "=== TIM3 Encoder Test Start ===\r\n");
    vofa_print(&huart1, "PA6: A相, PA7: B相, PB0: Z相\r\n\r\n");
    
    while (1)
    {
        /* 读取当前编码器计数 */
        current_count = __HAL_TIM_GET_COUNTER(&htim3);
        
        /* 计算增量（处理溢出） */
        delta = (int16_t)(current_count - last_count);
        
        /* 更新总位置 */
        encoder_position += delta;
        
        /* 保存当前计数 */
        last_count = current_count;
        
        /* 每100ms打印一次数据 */
        if (test_count % 10 == 0)
        {
            /* 判断旋转方向 */
            const char* direction = "STOP";
            if (delta > 0) 
                direction = "CW ";  /* 顺时针 */
            else if (delta < 0) 
                direction = "CCW";  /* 逆时针 */
            
            vofa_print(&huart1, 
                       "Count: %5u, %6ld, %5d, %s",
                       current_count, encoder_position, delta, direction);
            
            /* 如果捕获到Z相脉冲，显示捕获值 */
            if (ic3_capture_flag)
            {
                vofa_print(&huart1, " | Z-Pulse: %lu", ic3_capture_value);
                ic3_capture_flag = 0;
            }
            
            vofa_print(&huart1, "\r\n");
        }
        
        HAL_Delay(10);
        test_count++;
    }
}


