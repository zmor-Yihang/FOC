# FIFOFAST 使用指南

## 项目简介

**fifofast** 是一个专为微控制器（MCU）设计的高性能、通用型FIFO（先进先出）缓冲区库。它特别适用于需要在实时环境中处理数据输入/输出的应用场景，如串口数据缓冲、ADC采样结果存储等。

### 主要特性

- ✅ **支持任意数据类型**：支持所有C语言数据类型，包括自定义的typedef类型
- ✅ **静态内存分配**：无动态内存管理开销，适合资源受限的嵌入式系统
- ✅ **内联函数支持**：提高执行速度，特别适合在中断服务程序（ISR）中使用
- ✅ **最小RAM占用**：典型FIFO仅需3字节管理开销
- ✅ **易于使用**：提供完整的FIFO操作接口
- ✅ **良好的文档**：源代码中包含详尽的注释

### 限制说明

- **FIFO大小**：限制为**2^n**个元素，其他大小会自动向上取整
- **元素大小**：可**指针访问**的FIFO最大元素大小为**255字节**
- **程序存储器**：由于使用宏和内联函数，Flash使用量会略高于普通函数实现

---

## 快速开始

### 基本使用步骤

1. **包含头文件**
```c
#include "fifofast.h"
```

2. **声明FIFO**（在.h文件或全局作用域）
```c
_fff_declare(uint8_t, fifo_uart_rx, 128);  // 声明一个能存储128个uint8_t的fifo_uart_rx
```

3. **初始化FIFO**（在.c文件中，仅一次）
```c
_fff_init(fifo_uart_rx);
```

4. **使用FIFO**
```c
// 写入数据
_fff_write(fifo_uart_rx, data);

// 读取数据
uint8_t data = _fff_read(fifo_uart_rx);

// 检查FIFO状态
if (!_fff_is_empty(fifo_uart_rx)) {
    // 处理数据
}
```

---

## API 参考手册

### 1. FIFO 声明和初始化

#### `_fff_declare(type, id, depth)`
声明一个FIFO结构

**参数：**
- `type`：数据类型（如 uint8_t, int16_t, 自定义结构体等）
- `id`：FIFO的标识符名称
- `depth`：FIFO深度（元素数量），必须是2的幂次方（4, 8, 16, 32, 64, 128, 256...）

**示例：**
```c
_fff_declare(uint8_t, fifo_uart_rx, 64);    // 声明64字节FIFO
_fff_declare(int16_t, fifo_adc, 128);       // 声明128个int16_t的FIFO
```

#### `_fff_declare_p(type, id, depth)`
声明可通过指针访问的FIFO（支持作为函数参数传递）

**示例：**
```c
_fff_declare_p(uint8_t, fifo_buffer, 64);
```

#### `_fff_declare_a(type, id, depth, size)`
声明FIFO数组

**参数：**
- `size`：数组大小

**示例：**
```c
_fff_declare_a(uint8_t, fifo_channels, 32, 4);  // 4个通道，每个32字节
```

#### `_fff_init(id)`
初始化FIFO（必须在使用前调用，仅调用一次）

**示例：**
```c
_fff_init(fifo_uart_rx);
```

#### `_fff_init_p(id)` / `_fff_init_a(id, arraysize)`
初始化可指针访问的FIFO或FIFO数组

---

### 2. 状态查询 API

#### `_fff_is_empty(id)`
检查FIFO是否为空

**返回：** 非0表示为空，0表示非空

#### `_fff_is_full(id)`
检查FIFO是否已满

**返回：** 非0表示已满，0表示未满

#### `_fff_mem_level(id)`
获取当前FIFO中的元素数量

**返回：** 当前存储的元素数量

#### `_fff_mem_free(id)`
获取FIFO剩余空间

**返回：** 可写入的元素数量

#### `_fff_mem_depth(id)`
获取FIFO的总容量（编译时常量）

**返回：** FIFO最大容量

#### `_fff_data_size(id)`
获取单个元素的字节大小（编译时常量）

**返回：** 每个元素占用的字节数

---

### 3. 数据写入 API

#### `_fff_write(id, newdata)`
安全写入数据（带满检查）

**参数：**
- `newdata`：要写入的数据

**说明：** 如果FIFO已满，数据会被丢弃

**示例：**
```c
_fff_write(fifo_uart_rx, received_byte);
```

#### `_fff_write_lite(id, newdata)`
快速写入数据（不检查是否满）

**注意：** 使用前必须确保FIFO未满，否则会覆盖旧数据

**示例：**
```c
if (!_fff_is_full(fifo_uart_rx)) {
    _fff_write_lite(fifo_uart_rx, data);
}
```

#### `_fff_write_multiple(id, newdata, n)`
批量写入数组数据

**参数：**
- `newdata`：数据数组指针
- `n`：要写入的元素数量

**示例：**
```c
uint8_t buffer[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
_fff_write_multiple(fifo_uart_rx, buffer, 10);
```

#### `_fff_add(id)` / `_fff_add_lite(id)`
添加空间并返回指针（用于直接写入）

**返回：** 指向新分配空间的指针，如果满则返回NULL

**示例：**
```c
uint8_t* ptr = _fff_add(fifo_uart_rx);
if (ptr != NULL) {
    *ptr = data;  // 直接写入
}
```

---

### 4. 数据读取 API

#### `_fff_read(id)`
安全读取并移除数据（带空检查）

**返回：** 读取的数据，如果为空返回0

**示例：**
```c
uint8_t data = _fff_read(fifo_uart_rx);
```

#### `_fff_read_lite(id)`
快速读取并移除数据（不检查是否空）

**注意：** 使用前必须确保FIFO非空

**示例：**
```c
if (!_fff_is_empty(fifo_uart_rx)) {
    uint8_t data = _fff_read_lite(fifo_uart_rx);
}
```

#### `_fff_peek(id, idx)`
查看数据但不移除（可用于读写）

**参数：**
- `idx`：偏移量，0表示最早的数据

**示例：**
```c
// 读取
uint8_t first = _fff_peek(fifo_uart_rx, 0);    // 第一个元素
uint8_t second = _fff_peek(fifo_uart_rx, 1);   // 第二个元素

// 修改
_fff_peek(fifo_uart_rx, 0) = new_value;
```

---

### 5. 数据管理 API

#### `_fff_reset(id)`
清空FIFO

**示例：**
```c
_fff_reset(fifo_uart_rx);
```

#### `_fff_remove(id, amount)`
移除指定数量的元素（带检查）

**参数：**
- `amount`：要移除的元素数量

**示例：**
```c
_fff_remove(fifo_uart_rx, 10);  // 移除10个元素
```

#### `_fff_remove_lite(id, amount)`
快速移除元素（不检查）

**注意：** amount必须 <= 当前元素数量

#### `_fff_rebase(id)`
重组内部数组使第一个元素位于物理索引0

**用途：** 简化对FIFO中字符串的操作

---

## STM32 应用示例

### 示例 1：UART 接收缓冲区

这是最常见的应用场景，在中断中接收数据，在主循环中处理。

```c
/* main.h */
#include "fifofast.h"

// 声明256字节的UART接收FIFO
_fff_declare(uint8_t, uart_rx_fifo, 256);

/* main.c */
#include "main.h"

// 初始化FIFO
_fff_init(uart_rx_fifo);

UART_HandleTypeDef huart1;

// UART接收完成中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t received_byte;
        
        // 从UART读取数据
        HAL_UART_Receive(&huart1, &received_byte, 1, 0);
        
        // 写入FIFO（中断中使用，非常快速）
        _fff_write_lite(uart_rx_fifo, received_byte);
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &received_byte, 1);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();
    
    uint8_t rx_byte;
    
    // 启动UART接收中断
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    
    while (1)
    {
        // 主循环处理接收数据
        if (!_fff_is_empty(uart_rx_fifo))
        {
            uint8_t data = _fff_read(uart_rx_fifo);
            
            // 处理接收的数据
            ProcessReceivedData(data);
        }
        
        // 其他任务...
    }
}
```

---

### 示例 2：UART 发送缓冲区

使用FIFO作为发送缓冲区，实现非阻塞发送。

```c
/* uart_tx.h */
#include "fifofast.h"

_fff_declare(uint8_t, uart_tx_fifo, 256);

void UART_SendByte(uint8_t data);
void UART_SendString(const char* str);
void UART_ProcessTransmit(void);

/* uart_tx.c */
#include "uart_tx.h"

_fff_init(uart_tx_fifo);

extern UART_HandleTypeDef huart1;
static uint8_t tx_busy = 0;

// 添加字节到发送队列
void UART_SendByte(uint8_t data)
{
    // 等待FIFO有空间（或使用超时机制）
    while (_fff_is_full(uart_tx_fifo));
    
    // 写入FIFO
    _fff_write(uart_tx_fifo, data);
}

// 发送字符串
void UART_SendString(const char* str)
{
    while (*str)
    {
        UART_SendByte(*str++);
    }
}

// 在主循环或定时器中调用，处理发送
void UART_ProcessTransmit(void)
{
    // 如果正在发送，跳过
    if (tx_busy)
        return;
    
    // 如果FIFO有数据且UART空闲
    if (!_fff_is_empty(uart_tx_fifo))
    {
        uint8_t data = _fff_read(uart_tx_fifo);
        tx_busy = 1;
        HAL_UART_Transmit_IT(&huart1, &data, 1);
    }
}

// UART发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_busy = 0;
    }
}

/* 使用示例 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();
    
    while (1)
    {
        // 发送数据
        UART_SendString("Hello, STM32!\r\n");
        HAL_Delay(1000);
        
        // 处理发送队列
        UART_ProcessTransmit();
    }
}
```

---

### 示例 3：ADC 多通道采样缓冲

使用FIFO数组为多个ADC通道分别缓冲数据。

```c
/* adc_buffer.h */
#include "fifofast.h"

#define ADC_CHANNELS 4

// 为4个ADC通道各声明一个128深度的FIFO
_fff_declare_a(uint16_t, adc_fifo, 128, ADC_CHANNELS);

void ADC_Init(void);
void ADC_ProcessData(void);

/* adc_buffer.c */
#include "adc_buffer.h"

// 初始化FIFO数组
_fff_init_a(adc_fifo, ADC_CHANNELS);

extern ADC_HandleTypeDef hadc1;
uint16_t adc_values[ADC_CHANNELS];

// ADC转换完成回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 读取各通道ADC值并存入对应FIFO
    for (int ch = 0; ch < ADC_CHANNELS; ch++)
    {
        uint16_t value = HAL_ADC_GetValue(hadc);
        _fff_write(adc_fifo[ch], value);
    }
    
    // 启动下一次转换
    HAL_ADC_Start_IT(&hadc1);
}

// 主循环处理ADC数据
void ADC_ProcessData(void)
{
    for (int ch = 0; ch < ADC_CHANNELS; ch++)
    {
        if (!_fff_is_empty(adc_fifo[ch]))
        {
            uint16_t value = _fff_read(adc_fifo[ch]);
            
            // 处理该通道的数据
            ProcessChannelData(ch, value);
        }
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_ADC1_Init();
    
    // 启动ADC
    HAL_ADC_Start_IT(&hadc1);
    
    while (1)
    {
        ADC_ProcessData();
        HAL_Delay(10);
    }
}
```

---

### 示例 4：命令行解析器

使用FIFO和peek功能实现简单的命令行解析。

```c
/* command_parser.h */
#include "fifofast.h"

#define CMD_BUFFER_SIZE 256

_fff_declare(uint8_t, cmd_fifo, CMD_BUFFER_SIZE);

void CMD_Init(void);
void CMD_ProcessInput(void);

/* command_parser.c */
#include "command_parser.h"
#include <string.h>

_fff_init(cmd_fifo);

void CMD_Init(void)
{
    _fff_reset(cmd_fifo);
}

// 处理命令输入
void CMD_ProcessInput(void)
{
    // 查找换行符
    uint16_t level = _fff_mem_level(cmd_fifo);
    
    for (uint16_t i = 0; i < level; i++)
    {
        if (_fff_peek(cmd_fifo, i) == '\n' || _fff_peek(cmd_fifo, i) == '\r')
        {
            // 找到完整命令
            char cmd_buffer[64];
            uint16_t cmd_len = (i < sizeof(cmd_buffer)-1) ? i : sizeof(cmd_buffer)-1;
            
            // 提取命令
            for (uint16_t j = 0; j < cmd_len; j++)
            {
                cmd_buffer[j] = _fff_peek(cmd_fifo, j);
            }
            cmd_buffer[cmd_len] = '\0';
            
            // 移除已处理的命令（包括换行符）
            _fff_remove(cmd_fifo, i + 1);
            
            // 解析并执行命令
            ExecuteCommand(cmd_buffer);
            break;
        }
    }
}

// UART接收回调（接收字符到FIFO）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t rx_byte;
    
    if (huart->Instance == USART1)
    {
        _fff_write(cmd_fifo, rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void ExecuteCommand(const char* cmd)
{
    if (strcmp(cmd, "LED_ON") == 0)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
    else if (strcmp(cmd, "LED_OFF") == 0)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
    else if (strncmp(cmd, "DELAY ", 6) == 0)
    {
        uint32_t ms = atoi(cmd + 6);
        HAL_Delay(ms);
    }
}
```

---

### 示例 5：自定义数据结构的FIFO

FIFO可以存储任何数据类型，包括自定义结构体。

```c
/* sensor_data.h */
#include "fifofast.h"

// 定义传感器数据结构
typedef struct {
    uint32_t timestamp;
    float temperature;
    float humidity;
    float pressure;
} SensorData_t;

// 声明能存储64个传感器数据的FIFO
_fff_declare(SensorData_t, sensor_fifo, 64);

void Sensor_Init(void);
void Sensor_AddReading(float temp, float hum, float press);
void Sensor_ProcessData(void);

/* sensor_data.c */
#include "sensor_data.h"

_fff_init(sensor_fifo);

void Sensor_Init(void)
{
    _fff_reset(sensor_fifo);
}

// 添加传感器读数
void Sensor_AddReading(float temp, float hum, float press)
{
    SensorData_t data;
    data.timestamp = HAL_GetTick();
    data.temperature = temp;
    data.humidity = hum;
    data.pressure = press;
    
    if (!_fff_is_full(sensor_fifo))
    {
        _fff_write(sensor_fifo, data);
    }
}

// 处理传感器数据
void Sensor_ProcessData(void)
{
    while (!_fff_is_empty(sensor_fifo))
    {
        SensorData_t data = _fff_read(sensor_fifo);
        
        // 数据处理：日志记录、传输、计算等
        printf("Time: %lu, Temp: %.2f°C, Hum: %.2f%%, Press: %.2f hPa\r\n",
               data.timestamp, data.temperature, data.humidity, data.pressure);
    }
}

/* 定时器回调中采集数据 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        // 每秒采集一次
        float temp = ReadTemperature();
        float hum = ReadHumidity();
        float press = ReadPressure();
        
        Sensor_AddReading(temp, hum, press);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();
    
    Sensor_Init();
    HAL_TIM_Base_Start_IT(&htim2);
    
    while (1)
    {
        Sensor_ProcessData();
        HAL_Delay(100);
    }
}
```

---

### 示例 6：DMA + FIFO 高速数据采集

使用DMA采集数据到FIFO，实现高速连续采集。

```c
/* dma_acquisition.h */
#include "fifofast.h"

#define DMA_BUFFER_SIZE 128
#define FIFO_SIZE 1024

_fff_declare(uint16_t, data_fifo, FIFO_SIZE);

/* dma_acquisition.c */
#include "dma_acquisition.h"

_fff_init(data_fifo);

uint16_t dma_buffer[DMA_BUFFER_SIZE];
extern ADC_HandleTypeDef hadc1;

// DMA传输完成回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // DMA缓冲区已满，将数据批量写入FIFO
    _fff_write_multiple(data_fifo, dma_buffer, DMA_BUFFER_SIZE);
}

// DMA传输半完成回调（双缓冲模式）
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 处理前半部分数据
    _fff_write_multiple(data_fifo, dma_buffer, DMA_BUFFER_SIZE/2);
}

void StartHighSpeedAcquisition(void)
{
    // 启动DMA连续采集
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE);
}

void ProcessAcquisitionData(void)
{
    uint16_t buffer[128];
    uint16_t count = 0;
    
    // 批量读取数据
    while (!_fff_is_empty(data_fifo) && count < 128)
    {
        buffer[count++] = _fff_read(data_fifo);
    }
    
    if (count > 0)
    {
        // 处理数据：FFT、滤波、存储等
        ProcessSignal(buffer, count);
    }
}
```

---

## 注意事项与最佳实践

### 1. 中断安全性

如果FIFO在ISR和主循环中同时访问，必须确保原子性：

```c
// 在主循环中访问FIFO时禁用中断
__disable_irq();
uint8_t data = _fff_read(shared_fifo);
__enable_irq();
```

或使用CMSIS-RTOS的互斥量：

```c
osMutexWait(fifo_mutex, osWaitForever);
_fff_write(shared_fifo, data);
osMutexRelease(fifo_mutex);
```

### 2. 性能优化

- 在确定FIFO状态时，使用`_lite`版本函数可提高速度
- 使用`_fff_write_multiple()`批量写入比逐个写入更高效
- FIFO深度应为2的幂次方以获得最佳性能

### 3. 内存使用

不同深度的FIFO管理开销：

| 深度（元素） | 管理开销（字节） |
|------------|----------------|
| 4 - 128    | 3              |
| 256        | 4              |
| 512 - 32768| 6              |
| 65536      | 8              |

### 4. 调试技巧

在Atmel Studio或其他调试器中，可以直接查看FIFO结构：

```c
// 查看FIFO内部状态
// 在调试窗口添加以下表达式：
// fifo_uart_rx.read
// fifo_uart_rx.write
// fifo_uart_rx.level
// fifo_uart_rx.data
```

---

## 常见问题

### Q1: 如何选择FIFO大小？

**A:** 根据数据速率和处理速度估算。例如：
- UART 115200bps ≈ 11.5KB/s，如果每100ms处理一次，需要至少1.15KB缓冲
- 建议预留2-4倍余量，选择2048或4096

### Q2: 可以在多个.c文件中使用同一个FIFO吗？

**A:** 可以。在.h文件中声明，在一个.c文件中初始化，其他.c文件通过extern访问：

```c
// fifo_global.h
extern struct fff_fifo_uart_rx_s fifo_uart_rx;

// main.c
_fff_init(fifo_uart_rx);

// other.c
#include "fifo_global.h"
// 直接使用 fifo_uart_rx
```

### Q3: 如何处理FIFO满的情况？

**A:** 根据应用场景选择策略：
- **丢弃新数据**：使用`_fff_write()`（默认行为）
- **覆盖旧数据**：先`_fff_remove()`再`_fff_write()`
- **阻塞等待**：循环检查`_fff_is_full()`直到有空间

### Q4: FIFO能存储指针吗？

**A:** 可以，使用typedef：

```c
typedef uint8_t* BytePtr_t;
_fff_declare(BytePtr_t, ptr_fifo, 32);
```

---

## 移植到其他平台

fifofast主要依赖于：
- C99标准（`typeof`、复合语句）
- GCC编译器扩展

移植到STM32以外的平台（如ESP32、Nordic nRF、TI MSP430等）通常只需：

1. 复制`fifofast.h`和相关宏文件到项目
2. 包含头文件
3. 确保使用GCC或兼容编译器
4. 根据需要调整`FIFOFAST_MAX_DEPTH_POINTABLE`
