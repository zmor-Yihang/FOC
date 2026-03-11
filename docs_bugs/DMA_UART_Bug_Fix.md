# USART1 DMA 发送 Bug 修复总结

## 问题描述

在使用 STM32G431 的 USART1 配合 DMA 进行数据收发时，出现通信异常。

---

## Bug 原因分析

### 1. DMA 中断处理函数配置错误 ❌

**原代码：**
```c
/* DMA1通道3中断服务函数（USART1 RX DMA） */  // ← 注释就写错了
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);  // ❌ 错误：通道3是TX，却处理了RX
}
```

**问题：**
- DMA1_Channel3 被配置为 **USART1_TX**（发送）
- 但中断处理函数里调用的是 `&hdma_usart1_rx`（接收句柄）
- 导致 TX 发送完成中断无法正确处理，`huart1.gState` 永远不会被更新为 `HAL_UART_STATE_READY`

---

### 2. 缺少 DMA RX 通道中断配置 ❌

**原代码：**
```c
HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
// 缺少 DMA1_Channel4 的中断配置！
```

**问题：**
- DMA1_Channel4 被配置为 **USART1_RX**（接收）
- 但没有使能 `DMA1_Channel4_IRQn` 中断
- 也没有实现 `DMA1_Channel4_IRQHandler` 中断服务函数
- 导致 DMA 接收完成回调无法被触发

---

### 3. 发送函数缺少忙检测 ⚠️

**原代码：**
```c
void usart1_sendData(uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit_DMA(&huart1, data, size);  // 直接发送，不检查状态
}
```

**问题：**
- 如果上一次 DMA 发送还未完成，再次调用会导致数据覆盖或发送失败
- 与 `printf`（阻塞式发送）混用时可能产生竞争

---

## 解决方案

### 修复 1：修正 DMA 中断处理函数

```c
/* DMA1通道3中断服务函数（USART1 TX DMA） */
void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);  // ✅ 正确：处理TX句柄
}

/* DMA1通道4中断服务函数（USART1 RX DMA） */
void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);  // ✅ 新增：处理RX句柄
}
```

---

### 修复 2：添加 DMA RX 通道中断配置

```c
/* 配置DMA和USART1相关中断的优先级和使能 */
HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);  /* TX中断 */
HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);  /* ✅ 新增：RX中断 */
HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);           /* ✅ 新增 */
HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
HAL_NVIC_EnableIRQ(USART1_IRQn);
```

---

### 修复 3：添加发送忙等待

```c
void usart1_sendData(uint8_t *data, uint16_t size)
{
    /* ✅ 等待上一次DMA发送完成，避免冲突 */
    while (huart1.gState != HAL_UART_STATE_READY)
    {
        /* 可以添加超时处理 */
    }
    HAL_UART_Transmit_DMA(&huart1, data, size);
}
```

---

## DMA 通道与句柄对应关系

| DMA 通道 | 用途 | 句柄 | 中断处理函数 |
|---------|------|------|-------------|
| DMA1_Channel3 | USART1_TX | `hdma_usart1_tx` | `DMA1_Channel3_IRQHandler` |
| DMA1_Channel4 | USART1_RX | `hdma_usart1_rx` | `DMA1_Channel4_IRQHandler` |

---

## 经验教训

1. **DMA 通道与句柄必须一一对应** — 中断处理函数中传入的句柄必须与该通道配置的功能匹配
2. **使用 DMA 必须配置相应中断** — 否则 HAL 库状态机无法正确更新
3. **异步发送需要状态检查** — 在发送前检查上次发送是否完成，避免数据覆盖
4. **注释要准确** — 错误的注释可能误导调试方向

---

## 修复日期

2025-12-14
