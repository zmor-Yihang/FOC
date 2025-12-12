# STM32G4xx 驱动程序库 (Drivers)

## 📋 项目概述

本目录包含STM32G4微控制器的完整驱动程序库，分为两个主要部分：

1. **startup** - 低级启动代码和CMSIS标准接口（芯片初始化）
2. **driver_hal** - HAL高级驱动程序库和配置文件（外设驱动）

---

# STM32G4xx 启动文件 (Startup)

## 📋 项目概述

`startup` 文件夹包含STM32G4微控制器启动所需的核心文件，包括ARM CMSIS标准接口和STM32G4xx特定的启动代码。这些文件负责从硬件复位到用户应用的初始化过程。

## 📁 文件夹结构

```
startup/
├── inc_cmsis/                 # ARM CMSIS内核头文件
├── inc_stm32g4/              # STM32G4特定的头文件
└── src_stm32g4/              # STM32G4启动代码源文件
    ├── system_stm32g4xx.c
    ├── arm/
    │   └── startup_stm32g431xx.s
    └── gcc/
        └── startup_stm32g431xx.s
```

## 📌 inc_cmsis/ 目录 (ARM标准接口)

CMSIS(Cortex Microcontroller Software Interface Standard)是由ARM定义的标准接口，提供CPU内核级别的功能定义。

### 核心头文件

| 文件 | 用途 |
|------|------|
| `core_cm4.h` | **Cortex-M4核心定义** - 定义了Cortex-M4处理器的寄存器、中断、系统定时器等基本接口，包括NVIC、SysTick等 |
| `core_armv8mml.h` | ARMv8-M Mainline架构定义 |
| `core_armv8mbl.h` | ARMv8-M Baseline架构定义 |
| `core_armv81mml.h` | ARMv8.1-M Mainline架构定义 |

### 编译器支持头文件

| 文件 | 用途 |
|------|------|
| `cmsis_compiler.h` | **编译器兼容性** - 提供统一的编译器相关宏定义接口，屏蔽不同编译器的差异 |
| `cmsis_gcc.h` | GCC编译器特定宏定义 |
| `cmsis_armcc.h` | ARM Compiler特定宏定义 |
| `cmsis_armclang.h` | ARM Clang编译器特定宏定义 |
| `cmsis_armclang_ltm.h` | ARM Clang长期维护版本特定定义 |

### 内存管理与安全

| 文件 | 用途 |
|------|------|
| `mpu_armv7.h` | ARMv7内存保护单元(MPU)定义 - 用于内存保护和访问权限控制 |
| `mpu_armv8.h` | ARMv8内存保护单元(MPU)定义 |
| `tz_context.h` | TrustZone安全上下文管理 - 支持安全和非安全世界间的上下文切换 |

### 版本信息

| 文件 | 用途 |
|------|------|
| `cmsis_version.h` | CMSIS版本号定义 |

---

## 📌 inc_stm32g4/ 目录 (STM32G4特定定义)

包含STM32G4微控制器特有的外设寄存器定义和系统初始化声明。

### 重要头文件

| 文件 | 用途 |
|------|------|
| **`stm32g431xx.h`** | **STM32G431芯片特定定义** - 包含所有外设的寄存器地址、位定义、外设实例等。这是最重要的芯片级定义文件，定义了STM32G431的硬件特性，包括：<br/>- 所有外设的基地址<br/>- 寄存器位定义<br/>- 中断编号<br/>- 片上存储器大小<br/>- 外设可用性等 |
| **`system_stm32g4xx.h`** | **系统初始化接口声明** - 声明系统初始化函数和变量的原型，包括：<br/>- `SystemInit()` - 系统时钟初始化<br/>- `SystemCoreClockUpdate()` - 更新系统时钟变量<br/>- `SystemCoreClock` - 系统时钟频率变量<br/>用于在启动过程中配置系统时钟和PLL |

---

## 📌 src_stm32g4/ 目录 (启动源代码)

包含C语言和汇编语言编写的启动代码，负责从硬件复位到进入main()函数。

### 关键源文件

#### `system_stm32g4xx.c`

**功能**: 系统初始化的C语言实现

**关键内容**:
- **`SystemInit()`** - 系统初始化函数
  - 初始化系统时钟(HSI/HSE/PLL)
  - 配置FLASH访问时序
  - 初始化向量表偏移
  
- **`SystemCoreClockUpdate()`** - 更新系统时钟频率
  - 计算当前系统时钟频率
  - 更新 `SystemCoreClock` 全局变量
  
- **`SystemCoreClock`** - 全局变量
  - 存储系统时钟频率值(单位Hz)
  - 用于计算定时器、波特率等参数
  
- **时钟倍增因子定义**
  - `AHB_PRESCALER` - AHB分频系数
  - `APB1_PRESCALER` - APB1分频系数
  - `APB2_PRESCALER` - APB2分频系数
  - `PLL_M/N/P/Q` - PLL配置参数

**依赖关系**:
```
硬件复位 → 汇编启动代码 → SystemInit() → main()
```

---

### `arm/startup_stm32g431xx.s` 和 `gcc/startup_stm32g431xx.s`

**功能**: 低级启动代码(汇编语言)

**存在两个版本原因**:
- `arm/` - ARM Compiler (KEIL MDK)格式
- `gcc/` - GCC格式(用于STM32CubeIDE、GCC工具链)

**启动流程**:

1. **栈和堆初始化**
   - 设置栈指针(SP)为RAM顶部
   - 设置堆指针(heap)

2. **向量表设置**
   - 定义异常向量表(中断入口)
   - 包括复位、NMI、HardFault、SysTick等处理器异常

3. **数据段初始化**
   - 从FLASH复制初始化数据到RAM(.data段)
   - 清零未初始化的全局变量(.bss段)

4. **C库初始化**
   - 初始化C运行时环境
   - 调用全局对象构造函数(C++)

5. **调用SystemInit()**
   - 执行 `system_stm32g4xx.c` 中的系统初始化

6. **调用main()**
   - 跳转到用户应用入口

**关键符号**:
- `Reset_Handler` - 复位异常处理器入口
- `Default_Handler` - 未定义异常的默认处理器
- `NMI_Handler`, `HardFault_Handler` 等 - 异常处理函数

**典型的启动顺序**:
```
┌─────────────────────────────────────┐
│  硬件复位 (Reset Pin)               │
└────────────┬────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────┐
│  PC跳转到Reset_Handler              │  (汇编)              │
│  - 关闭中断                         │                      │
│  - 初始化栈指针SP                   │                      │
│  📁 arm/startup_stm32g431xx.s       │ (ARM Compiler)       │
│  📁 gcc/startup_stm32g431xx.s       │ (GCC工具链)          │
└────────────┬─────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────┐
│  初始化数据段(.data)和清零(.bss)    │  (汇编)              │
│  📁 arm/startup_stm32g431xx.s       │ (ARM Compiler)       │
│  📁 gcc/startup_stm32g431xx.s       │ (GCC工具链)          │
└────────────┬─────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────┐
│  调用 SystemInit()                  │  (C语言)             │
│  - 初始化系统时钟                   │                      │
│  - 配置PLL/分频器                   │                      │
│  - 配置FLASH等待周期                │                      │
│  📁 src_stm32g4/system_stm32g4xx.c  │ (系统初始化)         │
└────────────┬─────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────┐
│  初始化C运行时环境                  │  (汇编/C)            │
│  - 调用全局变量初始化               │                      │
│  - 调用构造函数(C++)                │                      │
│  📁 arm/startup_stm32g431xx.s       │ (ARM Compiler)       │
│  📁 gcc/startup_stm32g431xx.s       │ (GCC工具链)          │
└────────────┬─────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────┐
│  调用 main()                        │  (用户代码)          │
│  - 用户应用代码                     │                      │
│  📁 User/                           │ (用户应用目录)       │
└──────────────────────────────────────────────────────────────┘
```

---

## 🔧 使用说明

### 1. 选择正确的启动文件

根据使用的编译环境选择：
- **KEIL MDK** → 使用 `arm/startup_stm32g431xx.s`
- **STM32CubeIDE/GCC** → 使用 `gcc/startup_stm32g431xx.s`

### 2. 包含头文件

```c
#include "stm32g431xx.h"      // 芯片定义
#include "system_stm32g4xx.h" // 系统初始化声明
```

### 3. 修改系统时钟配置

编辑 `system_stm32g4xx.c`，修改以下参数：

```c
#define PLL_M    4    // PLL输入分频
#define PLL_N    85   // PLL倍频
#define PLL_P    2    // PLL P分频
#define PLL_Q    2    // PLL Q分频
#define PLL_R    2    // PLL R分频
```

### 4. 编译时的链接器配置

需要在链接器脚本中正确指定：
- 代码区FLASH起始地址
- RAM起始和结束地址
- 栈和堆的大小

---

## ⚙️ 常见问题

**Q: 为什么有两个startup_stm32g431xx.s文件？**
- A: 不同编译器的汇编语法略有不同。请根据你的IDE选择相应版本。

**Q: 可以修改SystemInit()吗？**
- A: 可以。如果需要自定义时钟配置，请修改 `system_stm32g4xx.c` 中的代码。

**Q: 堆栈溢出会发生什么？**
- A: 通常会导致HardFault异常。栈和堆大小在启动文件中定义，修改时要谨慎。

**Q: stm32g431xx.h和stm32g4xx_hal.h的区别？**
- A: 前者是芯片级寄存器定义，后者是HAL高级驱动接口。前者更底层、更轻量。

---

# STM32G4xx HAL 驱动程序库 (driver_hal)

## 📋 项目概述

`driver_hal` 文件夹包含STM32G4微控制器的硬件抽象层(HAL)驱动程序库。HAL提供了一个高级的、易于使用的接口来操作微控制器的各种外设（如GPIO、UART、SPI、I2C等）。

## 📁 文件夹结构

```
driver_hal/
└── inc/                      # HAL驱动头文件和配置文件
    ├── stm32g4xx_hal.h                  # 主HAL头文件
    ├── stm32g4xx_hal_conf_template.h    # HAL配置模板文件
    ├── stm32_assert_template.h          # 断言处理模板文件
    ├── stm32g4xx_hal_*.h                # 各外设HAL驱动头文件
    └── ...
```

## 📌 核心配置文件说明

### 1. `stm32g4xx_hal.h` - 主HAL头文件

**作用**:
- STM32G4xx HAL驱动库的主入口头文件
- 定义了HAL库的通用常量、宏和系统接口
- 包含所有其他HAL模块的头文件选择性包含

**主要内容**:
- **系统配置常量** - SysTick频率、VDD电压、缓存配置等
- **SYSCFG配置宏** - 内存重映射、快速I2C模式等系统功能配置
- **调试宏定义** - DBGMCU冻结/解冻外设的宏
- **条件编译** - 根据 `stm32g4xx_hal_conf.h` 中的宏定义来选择包含的模块

**使用方式**:
```c
#include "stm32g4xx_hal.h"  // 在你的应用代码中包含此文件
```

---

### 2. `stm32g4xx_hal_conf_template.h` - HAL配置文件

**作用**:
- 配置HAL库要启用的模块和功能
- 定义系统时钟、振荡器参数
- 控制是否启用某些外设驱动

**使用步骤**:

#### ① 复制并重命名文件
将此模板文件复制到你的应用项目目录，并重命名为 `stm32g4xx_hal_conf.h`:

```bash
cp Drivers/driver_hal/inc/stm32g4xx_hal_conf_template.h User/config/stm32g4xx_hal_conf.h
```

#### ② 选择启用的模块

在配置文件中，根据你的应用需要启用/禁用各个HAL模块：

```c
// 启用的模块
#define HAL_RCC_MODULE_ENABLED      // 时钟配置
#define HAL_GPIO_MODULE_ENABLED     // GPIO
#define HAL_UART_MODULE_ENABLED     // UART/USART
#define HAL_SPI_MODULE_ENABLED      // SPI
#define HAL_I2C_MODULE_ENABLED      // I2C

// 可以禁用不需要的模块以节省代码空间
// #define HAL_ADC_MODULE_ENABLED
// #define HAL_DAC_MODULE_ENABLED
```

#### ③ 配置系统参数

设置外部振荡器频率和系统工作参数：

```c
// 外部高速振荡器(HSE)频率
#if !defined  (HSE_VALUE)
#define HSE_VALUE    (8000000UL)    // 8MHz外部晶体
#endif

// 内部振荡器(HSI)频率
#if !defined  (HSI_VALUE)
#define HSI_VALUE    (16000000UL)   // STM32G4内部16MHz振荡器
#endif

// VDD电压配置
#define  VDD_VALUE                  (3300UL)  // 3.3V

// SysTick中断优先级
#define  TICK_INT_PRIORITY          (0x0FUL)  // 优先级15（最低）

// 缓存配置
#define  INSTRUCTION_CACHE_ENABLE   1U        // 启用指令缓存
#define  DATA_CACHE_ENABLE          1U        // 启用数据缓存
```

#### ④ 启用完整断言检查（可选）

取消下面这行的注释来启用HAL参数检查：

```c
#define USE_FULL_ASSERT  1U
```

启用后，HAL函数会检查输入参数的有效性，如果参数无效会调用 `assert_failed()` 函数。

---

### 3. `stm32_assert_template.h` - 断言处理文件

**作用**:
- 定义断言处理机制，用于HAL参数检查
- 当HAL函数接收到无效参数时调用，方便调试

**使用步骤**:

#### ① 复制并重命名文件

```bash
cp Drivers/driver_hal/inc/stm32_assert_template.h User/config/stm32_assert.h
```

#### ② 实现 `assert_failed()` 函数

在你的应用代码中实现此函数来处理断言失败：

```c
// 在你的某个 .c 文件中
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可以在这里添加自己的实现 */
  /* 例如：停止程序、打印错误信息、LED闪烁等 */
  printf("断言失败: %s, 第 %ld 行\r\n", file, line);
  
  // 停止程序运行
  while(1);
}
#endif  /* USE_FULL_ASSERT */
```

**断言宏的工作原理**:

```c
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
```

- 如果 `expr` 为真，什么都不做
- 如果 `expr` 为假，调用 `assert_failed()` 并传递文件名和行号

**示例** - HAL中的使用：

```c
HAL_StatusTypeDef HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  assert_param(IS_GPIO_PIN(GPIO_Pin));  // 检查管脚号是否有效
  assert_param(IS_GPIO_PIN_ACTION(PinState));  // 检查管脚状态是否有效
  
  /* 执行操作 */
  // ...
}
```

---

## 🔧 完整配置步骤

### 第1步：复制配置文件到项目中

```bash
# 假设项目结构为 User/config/
mkdir -p User/config

cp Drivers/driver_hal/inc/stm32g4xx_hal_conf_template.h User/config/stm32g4xx_hal_conf.h
cp Drivers/driver_hal/inc/stm32_assert_template.h User/config/stm32_assert.h
```

### 第2步：编辑配置文件

编辑 `User/config/stm32g4xx_hal_conf.h`：
- 启用需要的HAL模块
- 配置系统时钟参数
- 设置中断优先级

### 第3步：在代码中包含头文件

```c
// 在你的main.c或公共头文件中
#include "stm32g4xx_hal.h"
#include "stm32_assert.h"  // 如果启用了USE_FULL_ASSERT
```

### 第4步：实现断言处理函数（可选）

如果启用了 `USE_FULL_ASSERT`，在某个 .c 文件中实现 `assert_failed()` 函数。

### 第5步：编译时包含配置文件目录

在编译选项中添加配置文件的目录：

```bash
# GCC编译示例
gcc -I./User/config ...
```

---

## 📌 各外设HAL驱动文件

`driver_hal/inc/` 目录包含以下外设驱动头文件：

| 外设类型 | 头文件 | 用途 |
|---------|--------|------|
| **时钟管理** | `stm32g4xx_hal_rcc.h` | RCC（重置和时钟控制） - 系统时钟配置 |
| **GPIO** | `stm32g4xx_hal_gpio.h` | GPIO - 通用输入输出 |
| **中断** | `stm32g4xx_hal_exti.h` | EXTI - 外部中断 |
| **定时器** | `stm32g4xx_hal_tim.h` | TIM - 通用定时器 |
| **串行通信** | `stm32g4xx_hal_uart.h` | UART - 异步串行通信 |
| | `stm32g4xx_hal_usart.h` | USART - 同步/异步串行通信 |
| | `stm32g4xx_hal_spi.h` | SPI - 串行外围接口 |
| | `stm32g4xx_hal_i2c.h` | I2C - 内部集成电路 |
| **模数转换** | `stm32g4xx_hal_adc.h` | ADC - 模数转换器 |
| **数模转换** | `stm32g4xx_hal_dac.h` | DAC - 数模转换器 |
| **DMA** | `stm32g4xx_hal_dma.h` | DMA - 直接内存访问 |
| **其他** | 更多驱动... | PWM、RTC、看门狗等 |

每个外设的详细用法请参考对应的HAL驱动头文件中的函数声明和注释。

---

## ⚙️ 常见问题

**Q: stm32g4xx_hal_conf.h 文件为什么是模板？**
- A: 因为每个项目可能需要不同的配置。将模板复制到项目后，你可以根据实际需求自定义配置。

**Q: 禁用未使用的模块有什么好处？**
- A: 减少编译的代码大小，加快编译速度。如果不需要某个外设，禁用它可以节省存储空间。

**Q: 如何在HAL函数调用时检查错误？**
- A: 大多数HAL函数返回 `HAL_StatusTypeDef`，包括 `HAL_OK`、`HAL_ERROR` 等状态。总是检查返回值。

**Q: 为什么 HAL 初始化慢？**
- A: HAL提供了完整的功能和错误检查，牺牲了一些性能。如果需要高性能，可以使用LL（低层）驱动。

**Q: 可以同时使用HAL和LL驱动吗？**
- A: 可以，但需要小心。不同的驱动可能会相互干扰，特别是在配置同一外设时。
`