# STM32G431 FOC 电机控制项目

基于 STM32G431KBT6 直流无刷电机 (BLDC) 磁场定向控制 (FOC) 项目，支持有感/无感/弱磁控制模式。

## 功能特性

- Clark / Park 正反变换
- SVPWM 空间矢量调制，扇区法/零序电流注入法
- PI 电流环 / 速度环控制
- 弱磁控制
- 无感观测器
  - Luenberger 龙伯格观测器 + PLL
  - SMO 滑模观测器 + PLL
- 多种运行模式
  - I/F 开环启动
  - 电流闭环
  - 速度闭环 (有感)
  - 速度闭环 (Luenberger 无感)
  - 速度闭环 (SMO 无感)
  - 弱磁速度闭环

## 项目结构

```
User/
├── application/                    # 主程序入口
│   ├── main.c                      #   系统初始化 & 运行模式选择
│   └── main.h
├── bsp/                            # 板级支持包 (BSP)
│   ├── adc.c/h                     #   ADC 注入组采样 (TIM1 触发, 双电流)
│   ├── as5047.c/h                  #   AS5047P 磁编码器 SPI 驱动
│   ├── tim.c/h                     #   TIM1 三相互补 PWM / TIM3 速度计算
│   ├── spi.c/h                     #   SPI 底层驱动
│   ├── usart.c/h                   #   USART1 串口 (DMA + FIFO)
│   ├── clock.c/h                   #   系统时钟配置 (170MHz)
│   ├── led.c/h                     #   LED 指示灯
│   └── key.c/h                     #   按键 (非阻塞扫描)
├── foc/                            # FOC 核心算法层
│   ├── clark_park.c/h              #   Clark / Park 正反变换
│   ├── svpwm.c/h                   #   SVPWM 空间矢量调制
│   ├── pid.c/h                     #   PI 控制器 (带积分抗饱和)
│   ├── luenberger.c/h              #   Luenberger 龙伯格观测器 + PLL 锁相环
│   ├── smo.c/h                     #   SMO 滑模观测器 + PLL 锁相环
│   └── flux_weakening.c/h          #   弱磁控制 (电压环自动注入负 Id)
├── motor/                          # 电机运行模式 (应用层)
│   ├── if_open.c/h                 #   I/F 开环启动 (恒流 + 斜坡加速)
│   ├── current_closed.c/h          #   电流闭环 (Id/Iq 双环)
│   ├── speed_closed.c/h            #   速度闭环 (编码器有感)
│   ├── flux_weak_speed_closed.c/h  #   弱磁速度闭环 (编码器有感)
│   ├── sensorless_luenberger.c/h   #   无感闭环 (I/F 启动 → Luenberger 切换)
│   ├── sensorless_smo.c/h          #   无感闭环 (I/F 启动 → SMO 切换)
│   ├── speed_closed_with_luenberger.c/h  # 有感速度闭环 + Luenberger 观测对比
│   └── speed_closed_with_smo.c/h         # 有感速度闭环 + SMO 观测对比
├── test/                           # 各模块单元测试
│   ├── test_clark_park             #   坐标变换验证
│   ├── test_svpwm                  #   SVPWM 输出验证
│   ├── test_pid                    #   PI 控制器验证
│   ├── test_adc / test_as5047      #   ADC 采样 / 编码器读取测试
│   └── test_tim1 / test_led / test_key   # 外设功能测试
└── utils/                          # 通用工具库
    ├── fast_sin_cos.h              #   快速三角函数
    ├── fifofast.h                  #   FIFO 环形缓冲区
    ├── ramp.c/h                    #   斜坡函数
    ├── delay.c/h                   #   微秒延时
    └── print.c/h                   #   串口格式化打印
Drivers/                            # STM32 HAL 库 & CMSIS
Simulink_funtion/                   # MATLAB/Simulink 算法仿真脚本
python_tools/                       # Python 辅助计算工具
docs_bugs/                          # BUG 记录与修复文档
docs_notes/                         # 开发笔记
```

## 开发环境

- IDE: VS Code + [EIDE](https://github.com/github0null/eide) 嵌入式开发插件
- 工具链: ARM GCC
- 调试器: OpenOCD

## 编译与烧录

项目使用 EIDE 管理构建，VS Code 中可直接使用以下 Task:

| 操作 | Task |
|------|------|
| 编译 | `build` |
| 重新编译 | `rebuild` |
| 烧录 | `flash` |
| 编译并烧录 | `build and flash` |
| 清理 | `clean` |

## 快速开始

1. 克隆项目，用 VS Code 打开工作区 `FOC.code-workspace`
2. 安装 EIDE 插件，配置 ARM GCC 工具链路径
3. 在 `main.c` 中选择运行模式，例如:
   ```c
   sensorless_luenberger_init(2000); // Luenberger 无感，目标转速 2000 RPM
   ```
4. 执行 `build and flash` 任务编译并烧录
5. vofa+ 上位机查看波形

## 开发计划

- [x] SVPWM 空间矢量调制
- [x] PI 电流环 (Id / Iq 双环)
- [x] I/F 开环启动
- [x] 有感速度闭环
- [x] Luenberger观测器
- [x] 滑模观测器
- [x] 无感闭环 (I/F → 观测器自动切换)
- [x] 弱磁控制
- [ ] 死区补偿
- [ ] 参数辨识
- [ ] 非线性磁链观测器
- [ ] 过流 / 过压保护
- [ ] 上位机调参工具