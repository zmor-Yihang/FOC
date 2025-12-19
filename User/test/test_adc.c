#include "test_adc.h"

/* ADC采样值结构体实例 */
static adc_values_t adc_values = {0};

/* 测试标志位 */
static uint8_t calibration_done = 0;

/**
 * @brief ADC测试函数
 * 测试功能：
 * 1. ADC规则组DMA连续采样（PA0-PA3）
 * 2. ADC注入组由TIM1 TRGO触发采样
 * 3. 定期打印采样结果到UART1
 */
void test_adc(void)
{
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();

    /* 首次运行时进行零点校准 */
    if (!calibration_done)
    {
        vofa_print(&huart1, "========== ADC测试开始 ==========\r\n");
        vofa_print(&huart1, "正在进行ADC零点校准...\r\n");
        
        /* 初始化ADC */
        adc1_init();
        
        /* 执行零点校准（5秒） */
        adc1_calibrate_zero(&adc_values);
        
        vofa_print(&huart1, "ADC零点校准完成！\r\n");
        vofa_print(&huart1, "Ia零点: %.3f V\r\n", adc_values.ia_offset);
        vofa_print(&huart1, "Ib零点: %.3f V\r\n", adc_values.ib_offset);
        vofa_print(&huart1, "Ic零点: %.3f V\r\n", adc_values.ic_offset);
        vofa_print(&huart1, "========================================\r\n\r\n");
        
        calibration_done = 1;
        last_print_time = current_time;
    }

    /* 每500ms打印一次ADC数据 */
    if (current_time - last_print_time >= 500)
    {
        last_print_time = current_time;

        /* ===== 规则组DMA采样数据 ===== */
        vofa_print(&huart1, "========== 规则组DMA采样 ==========\r\n");
        
        /* 显示原始ADC值 */
        vofa_print(&huart1, "原始ADC值:\r\n");
        vofa_print(&huart1, "  CH1(PA0): %4d  ", adc_regular_buf[0]);
        vofa_print(&huart1, "  CH2(PA1): %4d\r\n", adc_regular_buf[1]);
        vofa_print(&huart1, "  CH3(PA2): %4d  ", adc_regular_buf[2]);
        vofa_print(&huart1, "  CH4(PA3): %4d\r\n", adc_regular_buf[3]);
        
        /* 转换为物理量 */
        adc1_value_convert((uint16_t *)adc_regular_buf, &adc_values);
        
        /* 显示电流值 */
        vofa_print(&huart1, "\r\n相电流:\r\n");
        vofa_print(&huart1, "  Ia: %7.3f A\r\n", adc_values.ia);
        vofa_print(&huart1, "  Ib: %7.3f A\r\n", adc_values.ib);
        vofa_print(&huart1, "  Ic: %7.3f A\r\n", adc_values.ic);
        
        /* 显示母线电压 */
        vofa_print(&huart1, "\r\n母线电压:\r\n");
        vofa_print(&huart1, "  Udc: %6.2f V\r\n", adc_values.udc);

        /* ===== 注入组触发采样数据 ===== */
        vofa_print(&huart1, "\r\n========== 注入组触发采样 ==========\r\n");
        vofa_print(&huart1, "原始ADC值 (TIM1触发):\r\n");
        vofa_print(&huart1, "  CH1(PA0): %4d  ", adc_injected_buf[0]);
        vofa_print(&huart1, "  CH2(PA1): %4d\r\n", adc_injected_buf[1]);
        vofa_print(&huart1, "  CH3(PA2): %4d  ", adc_injected_buf[2]);
        vofa_print(&huart1, "  CH4(PA3): %4d\r\n", adc_injected_buf[3]);
        
        /* 转换注入组数据为物理量 */
        adc_values_t injected_values = {
            .ia_offset = adc_values.ia_offset,
            .ib_offset = adc_values.ib_offset,
            .ic_offset = adc_values.ic_offset
        };
        adc1_value_convert((uint16_t *)adc_injected_buf, &injected_values);
        
        vofa_print(&huart1, "\r\n注入组相电流:\r\n");
        vofa_print(&huart1, "  Ia: %7.3f A\r\n", injected_values.ia);
        vofa_print(&huart1, "  Ib: %7.3f A\r\n", injected_values.ib);
        vofa_print(&huart1, "  Ic: %7.3f A\r\n", injected_values.ic);
        vofa_print(&huart1, "  Udc: %6.2f V\r\n", injected_values.udc);
        
        vofa_print(&huart1, "=======================================\r\n\r\n");
    }
}
