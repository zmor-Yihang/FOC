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

    GPIO_InitTypeDef gpio_init_struct = {0};                    /* GPIO初始化结构体 */
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
    htim1.Init.Period = TIM1_PERIOD - 1;                          /* 自动重装载值 */
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;      /* 中心对齐模式1：仅向上计数时产生中断 */
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;            /* 时钟分频因子 */
    htim1.Init.RepetitionCounter = 1;                             /* 重复计数器：每次溢出都产生更新事件 */
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; /* 使能自动重装载预装载 */
    HAL_TIM_PWM_Init(&htim1);                                     /* 初始化TIM1 PWM模式 */

    /* 配置TIM1为主模式，触发ADC采样 */
    tim1_master_init_struct.MasterOutputTrigger = TIM_TRGO_UPDATE; /* 设置TRGO输出触发源为更新事件 */
    tim1_master_init_struct.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    tim1_master_init_struct.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &tim1_master_init_struct);

    /* 配置输出比较通道参数 */
    tim1_oc_init_struct.OCMode = TIM_OCMODE_PWM2;
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
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void tim1_set_duty(float duty1, float duty2, float duty3)
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

    // 计算比较值：compare = duty * TIM1_PERIOD
    uint32_t compare1 = (uint32_t)(duty1 * TIM1_PERIOD);
    uint32_t compare2 = (uint32_t)(duty2 * TIM1_PERIOD);
    uint32_t compare3 = (uint32_t)(duty3 * TIM1_PERIOD);

    /* 设置比较值 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, compare2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, compare3);
}

float tim1_get_duty(uint32_t channel)
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