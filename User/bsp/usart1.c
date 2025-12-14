#include "usart1.h"

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
#define RX_BUFFER_SIZE 128           /* 定义接收缓冲区大小为128字节 */
uint8_t rxBuffer[RX_BUFFER_SIZE];    /* 实际接收缓冲区 */
volatile uint16_t rxSize = 0;        /* 实际接收到的数据长度 */
volatile uint8_t rxCompleteFlag = 0; /* 接收完成标志 */

void usart1_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0}; /* GPIO初始化结构体 */

    /* 使能USART1、GPIOB、DMA1时钟 */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* 配置USART1 TX引脚PB6为复用推挽输出模式 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; /* 配置复用功能为USART1 */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 配置USART1 RX引脚PB7为复用推挽输出模式 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 初始化UART参数 */
    huart1.Instance = USART1;                                     /* 指定USART1外设 */
    huart1.Init.BaudRate = 115200;                                /* 波特率115200 */
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

    /* 启动DMA方式接收，准备接收128字节数据到rxBuffer */
    HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);

    /* 使能UART空闲中断（IDLE），用于变长包分包 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* 发送数据函数 */
void usart1_sendData(uint8_t *data, uint16_t size)
{
    /* 等待上一次DMA发送完成，避免冲突 */
    while (huart1.gState != HAL_UART_STATE_READY)
    {
        /* 可以添加超时处理 */
    }
    HAL_UART_Transmit_DMA(&huart1, data, size); /* DMA 方式发送数据 */
}

void usart1_receiveData(uint8_t *data, uint16_t size)
{
    HAL_UART_Receive_DMA(&huart1, data, size); /* 阻塞方式接收数据 */
}

/* UART1中断服务函数 */
void USART1_IRQHandler(void)
{
    /* 判断是否为IDLE中断（空闲线） */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);                               /* 清除IDLE中断标志 */
        HAL_UART_DMAStop(&huart1);                                        /* 停止当前DMA接收 */
        rxSize = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); /* 计算已接收字节数 */
        rxCompleteFlag = 1;                                               /* 标记接收完成 */
        HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);          /* 重新启动DMA接收 */
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