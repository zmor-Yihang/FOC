#include "usart.h"

/*----------------------重定向串口打印--------------------------*/
int __io_putchar(int ch)
{
    /* 使用HAL库函数发送单个字符，超时时间为1000ms */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
    return ch;
}

int _write(int file, char *ptr, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        __io_putchar(*ptr++);
    }
    return len;
}
/*------------------------------------------------------------*/

/* 定义句柄 */
UART_HandleTypeDef huart1;        /* 声明UART1句柄，用于HAL库操作USART1 */
DMA_HandleTypeDef hdma_usart1_rx; /* 声明USART1接收DMA句柄 */
DMA_HandleTypeDef hdma_usart1_tx; /* 声明USART1发送DMA句柄 */

/* 接收缓冲区 */
uint8_t rx_buf_temp[RX_BUF_TEMP_SIZE]; /* 临时接收缓冲区 */
volatile uint16_t rx_size = 0;         /* 一次接收到的数据长度 */

static uint16_t last_dma_pos = 0;      /* 上次DMA位置，用于计算增量 */

_fff_declare(uint8_t, fifo_uart_rx, 256); // 声明256字节FIFO
_fff_init(fifo_uart_rx);                  // 初始化FIFO

void usart1_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0}; /* GPIO初始化结构体 */

    /* 使能USART1、GPIOB、DMA1时钟 */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* 配置USART1 TX引脚PB6为复用推挽输出模式 */
    gpio_init_struct.Pin = GPIO_PIN_6;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF7_USART1; /* 配置复用功能为USART1 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    /* 配置USART1 RX引脚PB7为复用推挽输出模式 */
    gpio_init_struct.Pin = GPIO_PIN_7;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    /* 初始化UART参数 */
    huart1.Instance = USART1;                                     /* 指定USART1外设 */
    huart1.Init.BaudRate = 500000;                                /* 波特率500000 */
    huart1.Init.WordLength = UART_WORDLENGTH_8B;                  /* 8位数据位 */
    huart1.Init.StopBits = UART_STOPBITS_1;                       /* 1位停止位 */
    huart1.Init.Parity = UART_PARITY_NONE;                        /* 无校验 */
    huart1.Init.Mode = UART_MODE_TX_RX;                           /* 发送+接收模式 */
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;                  /* 无硬件流控 */
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;              /* 16倍过采样 */
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;     /* 1位采样 */
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;             /* 时钟分频器为1 */
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; /* 无高级功能初始化 */
    HAL_UART_Init(&huart1);                                       /* 初始化UART1 */

    /* 配置DMA参数用于USART1 TX */
    hdma_usart1_tx.Instance = DMA1_Channel3;                       /* 指定DMA1通道3 */
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;           /* 关联USART1发送请求 */
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;          /* 内存到外设 */
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;              /* 外设地址不自增 */
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;                  /* 内存地址自增 */
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; /* 外设字节对齐 */
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存字节对齐 */
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;                         /* 正常模式 */
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;              /* 高优先级 */
    HAL_DMA_Init(&hdma_usart1_tx);                                 /* 初始化DMA */

    /* 配置DMA参数用于USART1 RX */
    hdma_usart1_rx.Instance = DMA1_Channel4;                       /* 指定DMA1通道5 */
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;           /* 关联USART1接收请求 */
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;          /* 外设到内存 */
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;              /* 外设地址不自增 */
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;                  /* 内存地址自增 */
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; /* 外设字节对齐 */
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    /* 内存字节对齐 */
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;                       /* 循环模式 */
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;              /* 高优先级 */
    HAL_DMA_Init(&hdma_usart1_rx);                                 /* 初始化DMA */

    /* 将DMA句柄与UART句柄进行关联 */
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    /* 配置DMA和USART1相关中断的优先级和使能 */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0); /* 设置DMA1通道3(TX)中断优先级 */
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);         /* 使能DMA1通道3(TX)中断 */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0); /* 设置DMA1通道4(RX)中断优先级 */
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);         /* 使能DMA1通道4(RX)中断 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);        /* 设置USART1中断优先级 */
    HAL_NVIC_EnableIRQ(USART1_IRQn);                /* 使能USART1中断 */

    /* 启动DMA方式接收，准备接收128字节数据到rx_buf_temp */
    HAL_UART_Receive_DMA(&huart1, rx_buf_temp, RX_BUF_TEMP_SIZE);

    /* 使能UART空闲中断（IDLE），用于变长包分包 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* 发送数据函数 */
void usart1_send_data(uint8_t *data, uint16_t size)
{
    /* 等待上一次DMA发送完成，避免冲突 */
    while (huart1.gState != HAL_UART_STATE_READY)
        ;
    HAL_UART_Transmit_DMA(&huart1, data, size); /* DMA 方式发送数据 */
}

/* 从FIFO读取指定数量的数据 */
uint16_t usart1_read_data(uint8_t *buf, uint16_t max_size)
{
    uint16_t i;
    for (i = 0; i < max_size && !_fff_is_empty(fifo_uart_rx); i++)
    {
        buf[i] = _fff_read(fifo_uart_rx);
    }
    return i; /* 返回实际读取的字节数 */
}

/* 获取 fifo_uart_rx 中剩余空间 */
uint16_t usart1_get_available_buffer(void)
{
    return _fff_mem_free(fifo_uart_rx);
}

/* 检查fifo_uart_rx是否为空, 0表示非空，非0表示空 */
uint8_t usart1_fifo_is_empty(void)
{
    return _fff_is_empty(fifo_uart_rx);
}

/* UART1中断服务函数 */
void USART1_IRQHandler(void)
{
    /* 判断是否为IDLE中断（空闲线） */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); /* 清除IDLE中断标志 */

        /* 计算当前DMA位置 */
        uint16_t curr_dma_pos = RX_BUF_TEMP_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        if (curr_dma_pos != last_dma_pos)
        {
            if (curr_dma_pos > last_dma_pos)
            {
                /* 正常情况：新数据在 last_dma_pos 到 curr_dma_pos 之间 */
                rx_size = curr_dma_pos - last_dma_pos;
                _fff_write_multiple(fifo_uart_rx, &rx_buf_temp[last_dma_pos], rx_size);
            }
            else
            {
                /* DMA环绕：先写 last_dma_pos 到末尾，再写开头到 curr_dma_pos */
                rx_size = RX_BUF_TEMP_SIZE - last_dma_pos;
                _fff_write_multiple(fifo_uart_rx, &rx_buf_temp[last_dma_pos], rx_size);
                if (curr_dma_pos > 0)
                {
                    _fff_write_multiple(fifo_uart_rx, rx_buf_temp, curr_dma_pos);
                }
                rx_size += curr_dma_pos;
            }
            last_dma_pos = curr_dma_pos; /* 更新位置 */
        }
    }
    HAL_UART_IRQHandler(&huart1); /* 处理HAL库内部其他中断事件 */
}

/* DMA1通道3中断服务函数（USART1 TX DMA） */
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx); /* 调用HAL库DMA TX中断处理函数 */
}

/* DMA1通道4中断服务函数（USART1 RX DMA） */
void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx); /* 调用HAL库DMA RX中断处理函数 */
}