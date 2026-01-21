# FOC 电机控制系统

基于 STM32G431 的 FOC（磁场定向控制）电机驱动系统，支持多种控制模式和无传感器控制。本项目实现了从开环到闭环、从有传感器到无传感器的完整电机控制方案。

## 目录
- [项目特性](#项目特性)
- [硬件平台](#硬件平台)
- [目录结构](#目录结构)
- [核心算法详解](#核心算法详解)
- [控制模式说明](#控制模式说明)
- [快速开始](#快速开始)
- [参数配置](#参数配置)
- [开发计划](#开发计划)
- [技术文档](#技术文档)

## 项目特性

### 已实现功能

#### FOC 核心算法
- **Clarke/Park 坐标变换**
  - ABC → αβ 静止坐标系（Clarke 变换）
  - αβ → dq 旋转坐标系（Park 变换）
  - 支持正反变换，实现电流解耦控制

- **SVPWM 空间矢量调制**
  - 七段式 SVPWM、min-max零序分量注入
  - 母线电压利用率提升至 86.6%
  - 自动扇区判断和矢量合成
  - 输出占空比范围：0.0 ~ 1.0

- **PID 控制器**
  - 电流环 PI 控制（Id/Iq 双环）
  - 速度环 PI 控制
  - 积分抗饱和限幅
  - 输出限幅保护

#### 控制模式

1. **开环 V/F 控制**
   - 电压-频率开环控制
   - 适用于启动和低速运行
   - 无需传感器反馈

2. **开环 I-F 控制**
   - 电流-频率开环控制
   - 带电流环闭环，转速开环
   - 启动力矩更大，更平滑

3. **电流闭环控制**
   - Id/Iq 双电流环 PI 控制
   - 需要编码器反馈电角度
   - 电流响应快速准确

4. **速度闭环控制（带编码器）**
   - 三环控制：速度环 + 双电流环
   - AS5047 磁编码器位置反馈
   - 速度精度高，动态响应好

5. **无传感器速度闭环（SMO）**
   - SMO - PLL 估算电角度和速度
   - I-F 启动 + SMO 闭环切换
   - 无需编码器，降低成本
   - 支持中高速运行

#### 硬件驱动（BSP 层）

- **AS5047 磁编码器**
  - SPI 通信，14-bit 分辨率
  - 角度读取精度：0.022°
  - 支持电角度/机械角度转换
  - 零点校准功能

- **三相电流采样**
  - ADC 注入组同步采样
  - 定时器触发，与 PWM 同步
  - 采样频率：10kHz
  - 支持 ABC 三相电流重构

- **高级定时器 PWM**
  - TIM1 生成三相互补 PWM
  - 频率：20kHz
  - 死区时间可配置
  - 支持刹车保护

- **串口调试**
  - USART1 用于 printf 输出
  - 支持 DMA 发送
  - 实时打印电流、速度、角度等信息

#### 工具库

- **高性能快速三角函数**
 * - Cody-Waite归约算法
 * - FMA 指令优化
 * - 多项式逼近方法

- **FIFO 缓冲区**
  - 高效环形缓冲区实现
  - 支持任意数据类型
  - 用于串口数据缓存

- **斜坡函数**
  - 平滑加减速控制
  - 可配置斜率
  - 防止电流突变

- **宏编程工具集**
  - 可变参数宏（NARG）
  - 宏拼接（CAT）
  - 数组操作宏
  - 类型安全宏

## 硬件平台

- **MCU**: STM32G431KBT6
  - ARM Cortex-M4F 内核
  - 主频：170MHz
  - Flash: 128KB, SRAM: 32KB
  - 硬件 FPU 支持

- **编码器**: AS5047P
  - 14-bit 磁编码器
  - SPI 接口
  - 分辨率：16384 位/圈

- **电源**
  - 母线电压：12V DC
  - 最大电流：2.5A

- **开发环境**
  - IDE: EIDE (Embedded IDE)
  - 编译器: OpenOCD

## 目录结构

```
├── User/
│   ├── application/     # 应用层
│   │   ├── main.c      # 主程序入口
│   │   └── motor_ctrl.c # 电机控制状态机
│   ├── bsp/            # 板级支持包（外设驱动）
│   │   ├── adc.c       # ADC 电流采样
│   │   ├── as5047.c    # 磁编码器驱动
│   │   ├── clock.c     # 时钟配置
│   │   ├── key.c       # 按键驱动
│   │   ├── led.c       # LED 驱动
│   │   ├── spi.c       # SPI 通信
│   │   ├── tim.c       # 定时器 PWM
│   │   └── usart.c     # 串口通信
│   ├── foc/            # FOC 算法核心
│   │   ├── clark_park.c # 坐标变换
│   │   ├── foc.c       # FOC 主控制逻辑
│   │   ├── pid.c       # PID 控制器
│   │   ├── smo.c       # 滑模观测器
│   │   └── svpwm.c     # SVPWM 调制
│   ├── motor/          # 电机控制模式
│   │   ├── if_open.c   # I-F 开环控制
│   │   ├── current_closed.c # 电流闭环
│   │   ├── speed_closed.c   # 速度闭环（编码器）
│   │   ├── speed_closed_with_smo.c # 速度闭环（SMO）
│   │   └── sensorless_smo.c # 无传感器启动
│   ├── test/           # 测试程序
│   │   ├── test_adc.c
│   │   ├── test_as5047.c
│   │   ├── test_svpwm.c
│   │   └── ...
│   └── utils/          # 工具库
│       ├── delay.c     # 延时函数
│       ├── fast_sin_cos.h # 快速三角函数
│       ├── fifofast.h  # FIFO 缓冲区
│       ├── ramp.c      # 斜坡函数
│       └── macros/     # 宏编程工具
├── Drivers/            # STM32 HAL 库
│   ├── hal/           # HAL 驱动
│   ├── startup/       # 启动文件
│   └── sys/           # 系统文件
└── docs/              # 技术文档与 BUG 修复记录
```

## 核心算法详解

### FOC 控制流程

```
电流采样 (ABC) 
    ↓
Clarke 变换 (ABC → αβ)
    ↓
Park 变换 (αβ → dq)
    ↓
PID 控制 (速度环 → Iq*, 电流环 → Vd/Vq)
    ↓
逆 Park 变换 (dq → αβ)
    ↓
SVPWM 调制 (αβ → PWM 占空比)
    ↓
三相 PWM 输出
```

### 坐标变换

**Clarke 变换**（三相静止坐标系 → 两相静止坐标系）
```
i_α = i_a
i_β = (i_a + 2*i_b) / √3
```

**Park 变换**（两相静止坐标系 → 两相旋转坐标系）
```
i_d = i_α * cos(θ) + i_β * sin(θ)
i_q = -i_α * sin(θ) + i_β * cos(θ)
```

### SVPWM 算法

- 将电压矢量分解到相邻两个基本矢量和零矢量
- 根据扇区计算各矢量作用时间
- 七段式调制，减少开关次数
- 输出三相占空比

### 滑模观测器（SMO）

**原理**：通过观测定子电流误差，估算反电动势，进而计算转子位置和速度

**核心方程**：
```
di/dt = (u - Rs*i - e) / Ls
```

**观测流程**：
1. 电流估算：根据电压和电流模型
2. 滑模控制：产生开关函数
3. 低通滤波：提取反电动势
4. PLL 锁相：计算角度和速度

## 控制模式说明

### 1. 开环 V/F 控制

**适用场景**：启动、低速运行、无传感器场合

**特点**：
- 无需位置反馈
- 控制简单
- 启动力矩小
- 易失步

**使用方法**：
```c
foc_open_loop_run(&foc_handle, 500, 2.0f); // 500 RPM, 2V
```

### 2. 开环 I-F 控制

**适用场景**：启动阶段、无传感器启动

**特点**：
- 电流环闭环，转速开环
- 启动力矩大
- 平滑加速
- 适合作为无传感器启动方式

**使用方法**：
```c
if_open_init(1000, 1.5f); // 1000 RPM, 1.5A
```

### 3. 电流闭环控制

**适用场景**：力矩控制、测试验证

**特点**：
- Id/Iq 独立控制
- 需要编码器反馈
- 电流响应快
- 可直接控制输出力矩

**使用方法**：
```c
current_closed_init(0.0f, 2.0f); // Id=0A, Iq=2A
```

### 4. 速度闭环控制（编码器）

**适用场景**：精确速度控制、高性能应用

**特点**：
- 三环控制结构
- 速度精度高
- 动态响应好
- 需要编码器

**使用方法**：
```c
speed_closed_init(3000); // 目标 3000 RPM
```

### 5. 无传感器速度闭环（SMO）

**适用场景**：低成本应用、中高速运行

**特点**：
- 无需编码器
- I-F 启动 + SMO 切换
- 适合中高速（>500 RPM）
- 低速性能较差

**使用方法**：
```c
sensorless_smo_init(3000); // 目标 3000 RPM
```

**启动流程**：
1. I-F 开环加速到 500 RPM
2. SMO 观测器开始工作
3. 切换到 SMO 闭环控制
4. 加速到目标转速

## 快速开始

### 硬件连接

1. 连接三相电机到驱动板
2. 连接 AS5047 编码器（可选）
3. 连接 12V 电源
4. 连接 ST-Link 调试器

### 编译与烧录

```bash
# 1. 使用 EIDE 打开项目
# 2. 选择编译配置
# 3. 点击编译按钮
# 4. 烧录到 STM32G431
```

### 运行示例

#### 示例 1：速度闭环（带编码器）

```c
int main(void)
{
    // 初始化硬件
    HAL_Init();
    clock_init();
    usart1_init();
    led1_init();
    key_init();
    as5047_init();
    tim1_init();
    adc1_init();
    
    // 启动速度闭环（目标 2000 RPM）
    speed_closed_init(2000);
    
    while (1)
    {
        // 按键停止
        if (key_scan() == 1)
        {
            printf("Stop!\n");
            break;
        }
        
        // 打印状态信息
        print_speed_info();
        HAL_Delay(100);
    }
}
```

#### 示例 2：无传感器速度闭环

```c
int main(void)
{
    // 初始化硬件
    HAL_Init();
    clock_init();
    usart1_init();
    adc1_init();
    tim1_init();
    
    // 启动无传感器速度闭环（目标 3000 RPM）
    sensorless_smo_init(3000);
    
    while (1)
    {
        if (key_scan() == 1) break;
        print_sensorless_info();
    }
}
```

#### 示例 3：电流闭环测试

```c
int main(void)
{
    // 初始化
    HAL_Init();
    clock_init();
    usart1_init();
    as5047_init();
    tim1_init();
    adc1_init();
    
    // 电流闭环（Id=0A, Iq=1.5A）
    current_closed_init(0.0f, 1.5f);
    
    while (1)
    {
        if (key_scan() == 1) break;
        print_current_info();
    }
}
```

## 参数配置

### 电机参数

在 `User/foc/foc.h` 中配置：

```c
#define U_DC 12.0f        // 直流母线电压 (V)
```

在 `User/bsp/as5047.h` 中配置：

```c
#define AS5047_MOTOR_POLE_PAIR 7  // 电机极对数
```

### PID 参数

电流环 PID（在各控制模式初始化函数中）：

```c
// Id 电流环
pid_init(&pid_id, 0.5f, 50.0f, -10.0f, 10.0f);
// Kp=0.5, Ki=50, 输出限幅 ±10V

// Iq 电流环
pid_init(&pid_iq, 0.5f, 50.0f, -10.0f, 10.0f);
```

速度环 PID：

```c
// 速度环
pid_init(&pid_speed, 0.01f, 0.1f, -5.0f, 5.0f);
// Kp=0.01, Ki=0.1, 输出限幅 ±5A
```

### SMO 参数

在 `User/motor/sensorless_smo.c` 中配置：

```c
smo_init(&smo,
    0.12f,      // Rs: 定子电阻 (Ω)
    0.0001f,   // Ls: 定子电感 (H)
    7,         // 极对数
    0.0001f,   // 采样周期 (s)
    50.0f,     // k_slide: 滑模增益
    0.95f,     // k_lpf: 低通滤波系数
    0.5f,      // boundary: 边界层厚度
    50.0f,     // fc: PLL 截止频率
    0.9f       // k_speed_lpf: 速度滤波系数
);
```

### 调试输出

通过串口打印实时信息：

```c
// 打印速度、电流、角度等信息
print_speed_info();
print_current_info();
print_sensorless_info();
```

## 开发计划

### 近期计划
- [ ] **串口命令解析**
  - 实时调整 PID 参数
  - 切换控制模式
  - 设置目标速度/电流
  
- [ ] **前馈解耦**
  - dq 轴交叉耦合补偿
  - 提升动态响应速度
  - 减少超调

- [ ] **死区补偿**
  - 补偿逆变器死区时间
  - 减少电流畸变
  - 提升低速性能

### 中期计划
- [ ] **位置闭环控制**
  - 位置环 + 速度环 + 电流环
  - 支持角度控制
  - 轨迹规划

- [ ] **弱磁控制**
  - 扩展高速运行范围
  - Id 注入控制
  - 最大转矩电流比（MTPA）

### 长期计划
- [ ] **滑模观测器优化**
  - 改进低速性能
  - 自适应滑模增益
  - 减少抖振

- [ ] **参数自整定**
  - 自动识别电机参数（Rs, Ls, Flux）
  - PID 参数自整定
  - 在线参数辨识

## 技术文档

详细的技术文档和问题修复记录请参考 `docs/` 目录：

- [ADC 注入组中断不触发 BUG](docs/ADC注入组中断不触发BUG.md)
- [DMA UART Bug 修复](docs/DMA_UART_Bug_Fix.md)
- [DMA UART FIFO Bug 修复](docs/DMA_UART_FIFO_Bug_Fix.md)
- [SVPWM 占空比分布不对称 BUG](docs/SVPWM占空比分布不对称BUG.md)
- [闭环退出时过流 BUG](docs/闭环退出时过流BUG.md)
- [BSP 层与应用层解耦](docs/BSP层与应用层解耦_回调函数指针.md)
- [FIFOFAST 使用指南](docs/FIFOFAST使用指南.md)
- [KEY 驱动非阻塞](docs/KEY驱动非阻塞.md)
- [Printf Float Bug 修复](docs/Printf_Float_Bug_Fix.md)
