#include "tim.h"

/* 高级定时器1句柄 */
TIM_HandleTypeDef htim1;

/* PA8-A10 为 TIM1_CH1-3, PB13-B15 为 TIM1_CHN 1-3 */
void tim1_init(void)
{
    /* 使能TIM1时钟 */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct = {0}; /* GPIO初始化结构体 */

    TIM_MasterConfigTypeDef tim1_master_init_struct = {0};      /* TIM1主从模式配置结构体 */
    TIM_OC_InitTypeDef tim1_oc_init_struct = {0};               /* TIM1输出比较配置结构体 */
    TIM_BreakDeadTimeConfigTypeDef tim1_bdtr_init_struct = {0}; /* TIM1刹车和死区配置结构体 */

    /* 配置PA8, PA9, PA10为TIM1通道输出 */
    gpio_init_struct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                /* 无上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF6_TIM1;         /* TIM1复用功能 */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    /* 配置PB13, PB14为TIM1互补通道输出 */
    gpio_init_struct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                /* 无上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF6_TIM1;         /* TIM1复用功能 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    /* 配置PB15为TIM1互补通道输出 */
    gpio_init_struct.Pin = GPIO_PIN_15;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                /* 无上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF4_TIM1;         /* TIM1复用功能 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    /* 配置TIM1基本参数 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = TIM1_PRESCALER;                        /* 预分频值 */
    htim1.Init.Period = TIM1_PERIOD;                              /* 自动重装载值：源码用8400，不是8400-1 */
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;      /* 中心对齐模式1：与源码一致 */
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            /* 时钟分频因子 */
    htim1.Init.RepetitionCounter = 1;                             /* 重复计数器：源码用1，每2次溢出更新一次 */
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /* 使能自动重装载预装载 */
    HAL_TIM_Base_Init(&htim1);                                    /* 先初始化Base */

    // sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    // HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htim1); /* 初始化TIM1 PWM模式 */

    /* 配置TIM1为主模式，触发ADC采样 */
    tim1_master_init_struct.MasterOutputTrigger = TIM_TRGO_UPDATE; /* 设置TRGO输出触发源为更新事件 */
    tim1_master_init_struct.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    tim1_master_init_struct.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &tim1_master_init_struct);

    /* 配置输出比较通道参数 */
    tim1_oc_init_struct.OCMode = TIM_OCMODE_PWM2;              /* PWM模式1：CNT<CCR时输出有效电平 */
    tim1_oc_init_struct.OCIdleState = TIM_OCIDLESTATE_RESET;   /* 主通道空闲状态为低电平 */
    tim1_oc_init_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET; /* 互补通道空闲状态为低电平 */
    tim1_oc_init_struct.OCPolarity = TIM_OCPOLARITY_HIGH;      /* 主通道输出极性为高 */
    tim1_oc_init_struct.OCNPolarity = TIM_OCNPOLARITY_HIGH;    /* 互补通道输出极性为高 */
    tim1_oc_init_struct.OCFastMode = TIM_OCFAST_DISABLE;       /* 禁用快速模式 */
    tim1_oc_init_struct.Pulse = TIM1_PERIOD / 2;               /* 初始占空比50% */
    HAL_TIM_PWM_ConfigChannel(&htim1, &tim1_oc_init_struct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &tim1_oc_init_struct, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &tim1_oc_init_struct, TIM_CHANNEL_3);

    /* 配置死区时间 */
    tim1_bdtr_init_struct.DeadTime = TIM1_DEADTIME;           /* 死区时间 */
    tim1_bdtr_init_struct.OffStateRunMode = TIM_OSSR_ENABLE;  /* 运行模式下关闭状态选择 */
    tim1_bdtr_init_struct.OffStateIDLEMode = TIM_OSSI_ENABLE; /* 空闲模式下关闭状态选择 */
    tim1_bdtr_init_struct.LockLevel = TIM_LOCKLEVEL_OFF;      /* 锁定级别 */
    tim1_bdtr_init_struct.BreakState = TIM_BREAK_DISABLE;     /* 禁用刹车输入 */
    tim1_bdtr_init_struct.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    tim1_bdtr_init_struct.BreakFilter = 0;
    tim1_bdtr_init_struct.Break2State = TIM_BREAK2_DISABLE;
    tim1_bdtr_init_struct.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    tim1_bdtr_init_struct.Break2Filter = 0;
    tim1_bdtr_init_struct.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* 使能自动输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &tim1_bdtr_init_struct);

    /* 启动PWM输出 */
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void tim1_set_pwm_duty(float duty1, float duty2, float duty3)
{
    /* 边界检查 */
    if (duty1 < 0.0f)
        duty1 = 0.0f;
    if (duty1 > 1.0f)
        duty1 = 1.0f;
    if (duty2 < 0.0f)
        duty2 = 0.0f;
    if (duty2 > 1.0f)
        duty2 = 1.0f;
    if (duty3 < 0.0f)
        duty3 = 0.0f;
    if (duty3 > 1.0f)
        duty3 = 1.0f;

    /* 计算比较值：compare = duty * TIM1_PERIOD */
    uint32_t compare1 = (uint32_t)(duty1 * TIM1_PERIOD);
    uint32_t compare2 = (uint32_t)(duty2 * TIM1_PERIOD);
    uint32_t compare3 = (uint32_t)(duty3 * TIM1_PERIOD);

    /* 设置比较值 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compare3);
}

float tim1_get_pwm_duty(uint32_t channel)
{
    switch (channel)
    {
    case TIM_CHANNEL_1:
        return (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) / TIM1_PERIOD;
    case TIM_CHANNEL_2:
        return (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2) / TIM1_PERIOD;
    case TIM_CHANNEL_3:
        return (float)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3) / TIM1_PERIOD;
    default:
        return 0.0f;
    }
}

/*-----------------------------------------TIM3-----------------------------------------------------*/

/* TIM3 编码器句柄 */
TIM_HandleTypeDef htim3;

/**
 * @brief TIM3编码器硬件初始化
 * @note  PA6: A相, PA7: B相, PB0: Z相
 */
void tim3_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_Encoder_InitTypeDef tim_encoder_init_struct = {0};
    TIM_MasterConfigTypeDef tim_master_init_struct = {0};
    TIM_IC_InitTypeDef tim_ic_init_struct = {0};

    /* 使能时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* 配置PA6、PA7为编码器通道A、B */
    gpio_init_struct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                /* 无上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;         /* TIM3复用功能 */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    /* 配置PB0为编码器Z相（输入捕获） */
    gpio_init_struct.Pin = GPIO_PIN_0;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                /* 无上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;         /* TIM3复用功能 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    /* TIM3 基本配置 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;                          /* 不分频 */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数 */
    htim3.Init.Period = 65535;                         /* 自动重装载值 */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* 时钟不分频 */
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_IC_Init(&htim3);

    /* 编码器模式配置 - 4倍频 */
    tim_encoder_init_struct.EncoderMode = TIM_ENCODERMODE_TI12; /* TI1 TI2 计数 */

    /* 通道1配置（编码器A相） */
    tim_encoder_init_struct.IC1Polarity = TIM_ICPOLARITY_RISING; /* 上升沿捕获 */
    tim_encoder_init_struct.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    tim_encoder_init_struct.IC1Prescaler = TIM_ICPSC_DIV1; /* 不分频 */
    tim_encoder_init_struct.IC1Filter = 0;                 /* 无滤波 */

    /* 通道2配置（编码器B相） */
    tim_encoder_init_struct.IC2Polarity = TIM_ICPOLARITY_RISING; /* 上升沿捕获 */
    tim_encoder_init_struct.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    tim_encoder_init_struct.IC2Prescaler = TIM_ICPSC_DIV1; /* 不分频 */
    tim_encoder_init_struct.IC2Filter = 0;                 /* 无滤波 */
    HAL_TIM_Encoder_Init(&htim3, &tim_encoder_init_struct);

    /* 主从模式配置 */
    tim_master_init_struct.MasterOutputTrigger = TIM_TRGO_RESET; /* TRGO输出触发源 */
    tim_master_init_struct.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &tim_master_init_struct);

    /* 通道3 输入捕获配置（Z相脉冲） */
    tim_ic_init_struct.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; /* 上升沿捕获 */
    tim_ic_init_struct.ICSelection = TIM_ICSELECTION_DIRECTTI;       /* 直接模式 */
    tim_ic_init_struct.ICPrescaler = TIM_ICPSC_DIV1;                 /* 不分频 */
    tim_ic_init_struct.ICFilter = 0;                                 /* 无滤波 */
    HAL_TIM_IC_ConfigChannel(&htim3, &tim_ic_init_struct, TIM_CHANNEL_3);

    /* 中断配置 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0); /* 中断优先级 */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);         /* 使能 TIM3 中断 */

    /* 启动编码器模式 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* 启动Z相输入捕获中断 */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
}

/* TIM3 中断服务程序 */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/**
 * @brief TIM3 输入捕获回调函数（Z相脉冲捕获）
 * @note 捕获到Z相脉冲时，可以用于零位校准
 *       如需使用，取消注释并在外部定义encoder对象
 */
// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim->Instance == TIM3)
//     {
//         if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
//         {
//             /* Z相脉冲捕获 - 可用于零位校准 */
//             /* 可选：重置计数器实现零位对齐 */
//             // __HAL_TIM_SET_COUNTER(&htim3, 0);
//         }
//     }
// }
