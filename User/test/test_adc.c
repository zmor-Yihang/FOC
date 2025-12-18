#include "test_adc.h"


/**
 * @brief ADC测试函数
 * 测试功能：
 * 1. ADC规则组DMA连续采样（PA0-PA3）
 * 2. ADC注入组由TIM1 TRGO触发采样
 * 3. 定期打印采样结果到UART1
 */
void test_adc(void)
{
    uint32_t test_count = 0;
    uint16_t regular_ch1, regular_ch2, regular_ch3, regular_ch4;
    uint16_t injected_ch1, injected_ch2, injected_ch3, injected_ch4;
    
    /* 初始化ADC */
    adc1_init();
    
    /* 启动ADC规则组DMA采样 */
    adc1_start_regular_dma();
    
    /* 启动ADC注入组中断采样 */
    adc1_start_injected();
    
    
    while (1)
    {
        /* 读取规则组DMA缓冲区数据 */
        regular_ch1 = adc_dma_buf[0];
        regular_ch2 = adc_dma_buf[1];
        regular_ch3 = adc_dma_buf[2];
        regular_ch4 = adc_dma_buf[3];
        
        /* 读取注入组中断采集的数据 */
        injected_ch1 = adc_injected_buf[0];
        injected_ch2 = adc_injected_buf[1];
        injected_ch3 = adc_injected_buf[2];
        injected_ch4 = adc_injected_buf[3];
        
        /* 打印采样数据到PC */
        vofa_print(&huart1, "Regular: %u, %u, %u, %u, %u, %u, %u, %u\r\n",
                   regular_ch1, regular_ch2, regular_ch3, regular_ch4,
                   injected_ch1, injected_ch2, injected_ch3, injected_ch4);
        
        HAL_Delay(500); /* 每500ms打印一次 */
        test_count++;
    }
}
