#include "adc.h"

/* ADC1句柄 */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* 规则组缓冲区 */
static uint16_t adc_regular_buf[4] = {0};

/* 注入组数据缓冲区 */
static uint16_t adc_injected_buf[4] = {0};

/* 三相电流零点补偿 */
static adc_offset_t adc_offset = {0};

/* 转换原始ADC值为实际电流和电压 */
static void adc1_value_convert(uint16_t *adc_buf, adc_values_t *adc_values_converted)
{
    float voltage;

    voltage = adc_buf[0] * 3.3f / 4096.0f - ADC_REF_VOLTAGE;
    adc_values_converted->ia = -ADC_CURRENT_SCALE * (voltage - adc_offset.ia_offset);

    voltage = adc_buf[1] * 3.3f / 4096.0f - ADC_REF_VOLTAGE;
    adc_values_converted->ib = -ADC_CURRENT_SCALE * (voltage - adc_offset.ib_offset);

    voltage = adc_buf[2] * 3.3f / 4096.0f - ADC_REF_VOLTAGE;
    adc_values_converted->ic = -ADC_CURRENT_SCALE * (voltage - adc_offset.ic_offset);

    adc_values_converted->udc = ADC_UDC_SCALE * (adc_buf[3] * 3.3f / 4096.0f);
}

/* adc1初始化 + 校准零点 */
void adc1_init(void)
{
    /* 使能时钟 */
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct = {0};            /* GPIO初始化结构体 */
    ADC_MultiModeTypeDef adc_multimode_struct = {0};    /* ADC多模式配置结构体 */
    ADC_InjectionConfTypeDef adc_injected_struct = {0}; /* ADC注入通道配置结构体 */
    ADC_ChannelConfTypeDef adc_channel_struct = {0};    /* ADC规则通道配置结构体 */

    /* 配置PA0, PA1, PA2, PA3为模拟输入模式 */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    /* 配置DMA */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;              /* 外设到内存 */
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;                  /* 外设地址不递增 */
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;                      /* 内存地址递增 */
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* 外设数据宽度16位 */
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* 内存数据宽度16位 */
    hdma_adc1.Init.Mode = DMA_CIRCULAR;                           /* 循环模式 */
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;             /* 非常高优先级 */
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1); /* 关联DMA到ADC */

    /* 配置ADC1基本参数 */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;              /* ADC时钟4分频 */
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;                        /* 12位分辨率 */
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;                        /* 数据右对齐 */
    hadc1.Init.GainCompensation = 0;                                   /* 无增益补偿 */
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;                         /* 使能扫描模式 */
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;                     /* 单次转换结束标志 */
    hadc1.Init.LowPowerAutoWait = DISABLE;                             /* 禁用低功耗自动等待 */
    hadc1.Init.ContinuousConvMode = ENABLE;                            /* 启用连续转换模式 */
    hadc1.Init.NbrOfConversion = 4;                                    /* 规则组4个通道 */
    hadc1.Init.DiscontinuousConvMode = DISABLE;                        /* 禁用间断模式 */
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;                  /* 软件触发 */
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; /* 上升沿触发 */
    hadc1.Init.DMAContinuousRequests = ENABLE;                         /* 使能DMA连续请求 */
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                     /* 数据溢出时覆写 */
    hadc1.Init.OversamplingMode = DISABLE;                             /* 禁用过采样 */
    HAL_ADC_Init(&hadc1);

    /* 配置ADC多模式为独立模式 */
    adc_multimode_struct.Mode = ADC_MODE_INDEPENDENT;
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &adc_multimode_struct);

    /* 配置规则通道1 - PA0/IN1 */
    adc_channel_struct.Channel = ADC_CHANNEL_1;
    adc_channel_struct.Rank = ADC_REGULAR_RANK_1;
    adc_channel_struct.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    adc_channel_struct.SingleDiff = ADC_SINGLE_ENDED;
    adc_channel_struct.OffsetNumber = ADC_OFFSET_NONE;
    adc_channel_struct.Offset = 0;
    HAL_ADC_ConfigChannel(&hadc1, &adc_channel_struct);

    /* 配置规则通道2 - PA1/IN2 */
    adc_channel_struct.Channel = ADC_CHANNEL_2;
    adc_channel_struct.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &adc_channel_struct);

    /* 配置规则通道3 - PA2/IN3 */
    adc_channel_struct.Channel = ADC_CHANNEL_3;
    adc_channel_struct.Rank = ADC_REGULAR_RANK_3;
    HAL_ADC_ConfigChannel(&hadc1, &adc_channel_struct);

    /* 配置规则通道4 - PA3/IN4 */
    adc_channel_struct.Channel = ADC_CHANNEL_4;
    adc_channel_struct.Rank = ADC_REGULAR_RANK_4;
    HAL_ADC_ConfigChannel(&hadc1, &adc_channel_struct);

    /* 配置注入通道1 - PA0/IN1 */
    adc_injected_struct.InjectedChannel = ADC_CHANNEL_1;
    adc_injected_struct.InjectedRank = ADC_INJECTED_RANK_1;
    adc_injected_struct.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5; /* 采样时间12.5周期 */
    adc_injected_struct.InjectedSingleDiff = ADC_SINGLE_ENDED;            /* 单端输入 */
    adc_injected_struct.InjectedOffsetNumber = ADC_OFFSET_NONE;           /* 无偏移 */
    adc_injected_struct.InjectedOffset = 0;
    adc_injected_struct.InjectedNbrOfConversion = 4;                                       /* 注入组4个通道 */
    adc_injected_struct.InjectedDiscontinuousConvMode = DISABLE;                           /* 禁用间断模式 */
    adc_injected_struct.AutoInjectedConv = DISABLE;                                        /* 禁用自动注入转换 */
    adc_injected_struct.QueueInjectedContext = DISABLE;                                    /* 禁用注入上下文队列 */
    adc_injected_struct.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;             /* TIM1 TRGO触发 */
    adc_injected_struct.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING; /* 上升沿触发 */
    adc_injected_struct.InjecOversamplingMode = DISABLE;                                   /* 禁用过采样 */
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &adc_injected_struct);

    /* 配置注入通道2 - PA1/IN2 */
    adc_injected_struct.InjectedChannel = ADC_CHANNEL_2;
    adc_injected_struct.InjectedRank = ADC_INJECTED_RANK_2;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &adc_injected_struct);

    /* 配置注入通道3 - PA2/IN3 */
    adc_injected_struct.InjectedChannel = ADC_CHANNEL_3;
    adc_injected_struct.InjectedRank = ADC_INJECTED_RANK_3;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &adc_injected_struct);

    /* 配置注入通道4 - PA3/IN4 */
    adc_injected_struct.InjectedChannel = ADC_CHANNEL_4;
    adc_injected_struct.InjectedRank = ADC_INJECTED_RANK_4;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &adc_injected_struct);

    /* 配置ADC中断优先级 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    /* ADC校准 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* 开启规则组转换进行电流零点对齐(电流采样补偿, 准确说不是校准) */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_regular_buf, 4);

    HAL_Delay(100);

    for (uint16_t i = 0; i < 5000; i++)
    {
        float ia_volt = adc_regular_buf[0] * 3.3f / 4096.0f - ADC_REF_VOLTAGE;
        float ib_volt = adc_regular_buf[1] * 3.3f / 4096.0f - ADC_REF_VOLTAGE;
        float ic_volt = adc_regular_buf[2] * 3.3f / 4096.0f - ADC_REF_VOLTAGE;

        adc_offset.ia_offset = adc_offset.ia_offset * 0.998f + ia_volt * 0.002f;
        adc_offset.ib_offset = adc_offset.ib_offset * 0.998f + ib_volt * 0.002f;
        adc_offset.ic_offset = adc_offset.ic_offset * 0.998f + ic_volt * 0.002f;

        HAL_Delay(1);
    }

    HAL_ADC_Stop_DMA(&hadc1);

    /* 开启注入组转换中断 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/* 获取三相电流偏移量 */
void adc1_get_offset(adc_offset_t *offsets)
{
    offsets->ia_offset = adc_offset.ia_offset;
    offsets->ib_offset = adc_offset.ib_offset;
    offsets->ic_offset = adc_offset.ic_offset;
}

/* 获取规则组转换值 */
void adc1_get_regular_values(adc_values_t *values)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_regular_buf, 4); /* 开启采样 */
    HAL_Delay(10);                                             /* 延时确保采样完成 */
    adc1_value_convert(adc_regular_buf, values);               /* 计数值转换为实际值 */
    HAL_ADC_Stop_DMA(&hadc1);
}

/* 获取注入组转换值 */
void adc1_get_injected_values(adc_values_t *values)
{
    adc1_value_convert(adc_injected_buf, values);
}

/* ADC注入组转换完成中断处理函数 */
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

/* ADC注入转换完成回调函数 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* 读取注入组转换结果 */
        adc_injected_buf[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        adc_injected_buf[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        adc_injected_buf[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        adc_injected_buf[3] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);

        current_closed_loop_handler();
        speed_closed_loop_handler();
    }
}
