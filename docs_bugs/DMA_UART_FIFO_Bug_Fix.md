# DMA + UART + FIFO 接收数据重复/不删除 Bug 修复记录

## 问题描述

使用 STM32 HAL 库配合 DMA 循环模式接收 UART 数据，通过 IDLE 空闲中断检测帧结束，将数据写入 FIFO 缓冲区。

**现象：**
1. 发送数据后无法正确回显
2. FIFO 读取数据后，数据没有被"删除"，出现重复数据
3. 多次发送后数据错乱

---

## 原因分析

### 错误的实现方式

原始代码在 IDLE 中断中直接使用 DMA 计数器计算接收长度：

```c
/* 错误代码 */
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        
        /* ❌ 问题1: rx_size 是累计值，不是本次增量 */
        rx_size = RX_BUF_TEMP_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        
        /* ❌ 问题2: 每次都从缓冲区开头写入，导致重复数据 */
        _fff_write_multiple(fifo_uart_rx, rx_buf_temp, rx_size);
    }
    HAL_UART_IRQHandler(&huart1);
}
```

### 问题根源

| 问题 | 说明 |
|------|------|
| **累计值 vs 增量值** | `RX_BUF_TEMP_SIZE - DMA_Counter` 得到的是从 DMA 启动以来的**累计接收字节数**，而不是本次 IDLE 中断新收到的字节数 |
| **循环模式下的位置追踪** | DMA 配置为 `DMA_CIRCULAR` 循环模式，缓冲区会循环覆盖。如果不记录上次位置，无法知道哪些是新数据 |
| **数据重复写入** | 每次中断都从 `rx_buf_temp[0]` 开始写入 `rx_size` 字节，导致旧数据被重复写入 FIFO |

### 举例说明

假设 `RX_BUF_TEMP_SIZE = 128`：

```
第1次发送 "ABC" (3字节):
  DMA Counter = 125, rx_size = 128 - 125 = 3
  写入 FIFO: rx_buf_temp[0..2] = "ABC" ✓ 看起来正确

第2次发送 "DE" (2字节):
  DMA Counter = 123, rx_size = 128 - 123 = 5  ← 累计值！
  写入 FIFO: rx_buf_temp[0..4] = "ABCDE"      ← 重复写入了 "ABC"！
```

---

## 解决方案

### 核心思路

1. **记录上次 DMA 位置**：用 `last_dma_pos` 变量保存上次处理到的位置
2. **计算增量**：`新数据长度 = 当前位置 - 上次位置`
3. **处理环绕**：循环模式下 DMA 会从末尾绕回开头，需要分两段处理

### 修复后的代码

#### 1. 添加位置追踪变量

```c
/* 接收缓冲区 */
uint8_t rx_buf_temp[RX_BUF_TEMP_SIZE]; /* 临时接收缓冲区 */
volatile uint16_t rx_size = 0;         /* 一次接收到的数据长度 */
static uint16_t last_dma_pos = 0;      /* ✅ 新增：上次DMA位置，用于计算增量 */

_fff_declare(uint8_t, fifo_uart_rx, 256);
_fff_init(fifo_uart_rx);
```

#### 2. 修改中断处理函数

```c
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
                /* ✅ 正常情况：新数据在 last_dma_pos 到 curr_dma_pos 之间 */
                rx_size = curr_dma_pos - last_dma_pos;
                _fff_write_multiple(fifo_uart_rx, &rx_buf_temp[last_dma_pos], rx_size);
            }
            else
            {
                /* ✅ DMA环绕：先写 last_dma_pos 到末尾，再写开头到 curr_dma_pos */
                rx_size = RX_BUF_TEMP_SIZE - last_dma_pos;
                _fff_write_multiple(fifo_uart_rx, &rx_buf_temp[last_dma_pos], rx_size);
                if (curr_dma_pos > 0)
                {
                    _fff_write_multiple(fifo_uart_rx, rx_buf_temp, curr_dma_pos);
                }
                rx_size += curr_dma_pos;
            }
            last_dma_pos = curr_dma_pos; /* ✅ 更新位置 */
        }
    }
    HAL_UART_IRQHandler(&huart1);
}
```

---

## 修复后的数据流示意

```
DMA 缓冲区 (128字节, 循环模式):
┌─────────────────────────────────────────────────────────┐
│ A B C D E F G H ...                                     │
└─────────────────────────────────────────────────────────┘
        ↑           ↑
   last_dma_pos  curr_dma_pos
        │←─ 新数据 ─→│

第1次发送 "ABC":
  last_dma_pos = 0, curr_dma_pos = 3
  增量 = 3 - 0 = 3
  写入: rx_buf_temp[0..2] = "ABC"
  更新: last_dma_pos = 3

第2次发送 "DE":
  last_dma_pos = 3, curr_dma_pos = 5
  增量 = 5 - 3 = 2
  写入: rx_buf_temp[3..4] = "DE"  ← 只写入新数据！
  更新: last_dma_pos = 5

环绕情况 (假设 last_dma_pos = 126, 发送 "XYZ"):
  curr_dma_pos = 1 (绕回开头)
  第一段: rx_buf_temp[126..127] = "XY"
  第二段: rx_buf_temp[0] = "Z"
  更新: last_dma_pos = 1
```

---

## 相关配置要点

### DMA 配置

```c
/* DMA 必须配置为循环模式 */
hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
```

### FIFO 读取（主循环）

```c
/* 从 FIFO 读取数据 - _fff_read 会自动移除已读数据 */
uint16_t usart1_read_data(uint8_t *buf, uint16_t max_size)
{
    uint16_t i;
    for (i = 0; i < max_size && !_fff_is_empty(fifo_uart_rx); i++)
    {
        buf[i] = _fff_read(fifo_uart_rx);  /* 读取并移除 */
    }
    return i;
}
```

---

## 总结

| 项目 | 错误实现 | 正确实现 |
|------|----------|----------|
| 数据长度计算 | 累计值 | 增量值 (curr - last) |
| 写入起点 | 始终从 `buf[0]` | 从 `buf[last_dma_pos]` |
| 位置追踪 | 无 | `last_dma_pos` 变量 |
| 环绕处理 | 无 | 分两段写入 |

---

## 参考

- STM32 HAL 库 UART DMA 接收
- FIFOFAST 库：https://github.com/nqtronix/fifofast
- IDLE 空闲中断用于变长帧检测
