# BSP层与应用层解耦 - 回调函数指针方案

## 问题背景

在 FOC 电机控制项目中，ADC 注入组中断需要调用电流/速度闭环处理函数。原始实现中，BSP 层（`adc.c`）直接包含并调用测试层的函数，导致**依赖方向错误**。

---

## 错误的依赖关系

```
┌─────────────────────────────────────┐
│  adc.h (BSP层 - 底层驱动)            │
│                                     │
│  #include "test/test_xxx.h"  ← ❌   │
└─────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│  adc.c 中断回调                      │
│                                     │
│  current_closed_loop_handler();  ← 直接调用上层函数
│  speed_closed_loop_handler();    ← 直接调用上层函数
└─────────────────────────────────────┘
```

**问题：**
- 底层驱动依赖上层应用代码
- 无法复用 BSP 层
- 违反分层架构原则

---

## 解决方案：回调函数指针

### 核心思想

BSP 层不直接调用具体函数，而是：
1. 定义一个**函数指针类型**
2. 提供**注册接口**让上层注册回调
3. 中断中通过**指针间接调用**

```
┌─────────────────────────────────────┐
│  adc.h (BSP层) - 不依赖任何上层代码   │
│                                     │
│  // 1. 定义函数指针类型               │
│  typedef void (*callback_t)(void);  │
│                                     │
│  // 2. 提供注册接口                   │
│  void adc1_register_callback(...);  │
└─────────────────────────────────────┘
                 ▲
                 │ 上层主动注册
                 │
┌─────────────────────────────────────┐
│  test_xxx.c (应用层)                 │
│                                     │
│  // 3. 注册自己的处理函数             │
│  adc1_register_callback(handler);   │
└─────────────────────────────────────┘
```

---

## 具体实现

### 1. adc.h - 定义接口

```c
#ifndef __ADC_H__
#define __ADC_H__

#include "stm32g4xx_hal.h"

/* 定义回调函数类型：无参数、无返回值的函数 */
typedef void (*adc_injected_callback_t)(void);

/* 注册回调函数的接口 */
void adc1_register_injected_callback(adc_injected_callback_t callback);

/* 其他 ADC 接口... */
void adc1_init(void);

#endif
```

### 2. adc.c - 实现回调机制

```c
#include "adc.h"

/* 静态变量：存储回调函数指针，初始为空 */
static adc_injected_callback_t adc_injected_callback = NULL;

/* 注册函数：保存外部传入的函数地址 */
void adc1_register_injected_callback(adc_injected_callback_t callback)
{
    adc_injected_callback = callback;
}

/* ADC 注入组中断回调 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* 读取 ADC 数据... */
        
        /* 通过指针调用注册的函数 */
        if (adc_injected_callback != NULL)
        {
            adc_injected_callback();  // 间接调用
        }
    }
}
```

### 3. test_speed_closed_loop.c - 注册回调

```c
#include "bsp/adc.h"  // 上层依赖下层 ✓

/* 处理函数改为 static，仅内部使用 */
static void speed_closed_loop_handler(void)
{
    /* 速度闭环控制逻辑... */
}

void test_speed_closed_loop(void)
{
    /* 初始化... */
    
    /* 注册回调：把自己的 handler 地址传给 BSP 层 */
    adc1_register_injected_callback(speed_closed_loop_handler);
    
    /* 使能控制环 */
    speed_loop_enable = 1;
    
    /* 主循环... */
}
```

---

## 执行流程图解

```
程序启动
    │
    ▼
┌─────────────────────────────────────────┐
│ adc_injected_callback = NULL            │  ← 初始状态：无回调
└─────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────┐
│ test_speed_closed_loop() 被调用          │
│                                         │
│ adc1_register_injected_callback(        │
│     speed_closed_loop_handler           │  ← 注册：保存函数地址
│ );                                      │
└─────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────┐
│ adc_injected_callback = 0x08001234      │  ← 指针指向 handler
│                          ↑              │
│            speed_closed_loop_handler    │
└─────────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────────┐
│ ADC 中断触发                             │
│                                         │
│ HAL_ADCEx_InjectedConvCpltCallback()    │
│     │                                   │
│     ▼                                   │
│ adc_injected_callback();                │  ← 通过指针调用
│     │                                   │
│     ▼                                   │
│ 实际执行 speed_closed_loop_handler()    │
└─────────────────────────────────────────┘
```

---

## 函数指针基础

```c
/* 普通变量：存储数据 */
int a = 10;

/* 函数指针：存储函数地址 */
void (*callback)(void);           // 声明指针变量
callback = my_function;           // 赋值（函数名就是地址）
callback();                       // 通过指针调用函数

/* 等价于直接调用 */
my_function();
```

### typedef 简化写法

```c
/* 不用 typedef */
void (*adc_injected_callback)(void);  // 每次都要写完整声明

/* 用 typedef */
typedef void (*adc_injected_callback_t)(void);  // 定义类型别名
adc_injected_callback_t callback;               // 简洁声明
```

---

## 依赖关系对比

| 项目 | 修改前 | 修改后 |
|------|--------|--------|
| adc.h 包含 | `#include "test/test_xxx.h"` | 无上层依赖 |
| adc.c 调用 | `current_closed_loop_handler()` 直接调用 | `callback()` 间接调用 |
| 依赖方向 | BSP → 测试层 ❌ | 测试层 → BSP ✓ |
| 可复用性 | 差 | 好 |

---

## 扩展：多回调支持

如果需要同时注册多个回调：

```c
/* adc.h */
#define ADC_MAX_CALLBACKS 4
void adc1_register_callback(adc_injected_callback_t cb);

/* adc.c */
static adc_injected_callback_t callbacks[ADC_MAX_CALLBACKS] = {NULL};
static uint8_t callback_count = 0;

void adc1_register_callback(adc_injected_callback_t cb)
{
    if (callback_count < ADC_MAX_CALLBACKS)
    {
        callbacks[callback_count++] = cb;
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 依次调用所有注册的回调 */
    for (uint8_t i = 0; i < callback_count; i++)
    {
        if (callbacks[i] != NULL)
        {
            callbacks[i]();
        }
    }
}
```

---

## 总结

| 概念 | 说明 |
|------|------|
| **函数指针** | 存储函数地址的变量，可以间接调用函数 |
| **回调函数** | 由上层定义、传递给下层、在特定时机被下层调用的函数 |
| **解耦目的** | 让底层代码不依赖上层，提高可复用性和可维护性 |
| **注册机制** | 上层主动把函数地址传给下层保存 |

这种模式在嵌入式开发中非常常见，如 HAL 库的各种回调函数（`HAL_UART_RxCpltCallback` 等）就是类似原理。

---

**修复日期：** 2025-01-07
