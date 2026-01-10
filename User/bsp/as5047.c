#include "as5047.h"

/* CS 引脚控制宏 */
#define AS5047_CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define AS5047_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

/**
 * @brief 速度计算相关的静态变量
 */
static struct
{
    uint16_t last_angle_raw;   /* 上一次的角度原始值 */
    float last_angle_rad;      /* 上一次的角度值 (rad) */
    float speed_rpm;           /* 原始转速 (RPM) - 未滤波 */
    float speed_rpm_lpf;       /* 低通滤波后的转速 (RPM) */
    int32_t theta_sum;         /* 角度增量累加值 (用于速度计算) */
    uint8_t update_cnt;        /* 更新计数器 (用于分频) */
    uint8_t is_initialized;    /* 初始化标志位 */
} as5047_speed_data = {0, 0.0f, 0.0f, 0.0f, 0, 0, 0};

/**
 * @brief 计算奇偶校验位
 * @param data 需要计算的数据 (15位)
 * @return 奇偶校验位 (0 或 1)
 */
static uint16_t as5047_calc_parity(uint16_t data)
{
    // 只校验低15位（bit0~bit14）
    data &= 0x7FFF; // 清除 bit15（虽然传入时通常为0）

    // 计算1的个数的奇偶性（返回1表示奇数个1）
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 1; // 1 = 奇数个1, 0 = 偶数个1
}

/**
 * @brief SPI 发送接收一个字 (16位)
 */
static uint16_t as5047_spi_transfer(uint16_t tx_data)
{
    uint16_t rx_data = 0;

    AS5047_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx_data, (uint8_t *)&rx_data, 1, 1);

    AS5047_CS_HIGH();

    return rx_data;
}

/**
 * @brief 读取 AS5047P 寄存器
 */
static uint16_t as5047_read_reg(uint16_t reg_addr)
{
    uint16_t cmd;
    uint16_t data;

    /* 构建读命令: bit14 = 1 (读操作) */
    cmd = reg_addr | 0x4000;

    /* 计算并添加奇偶校验位 (bit15) */
    if (as5047_calc_parity(cmd) == 1)
    {
        cmd |= 0x8000;
    }

    /* 发送读命令，忽略返回值 (返回的是上一次命令的结果) */
    as5047_spi_transfer(cmd);

    /* 发送 NOP 命令，获取实际数据 */
    data = as5047_spi_transfer(0xC000); /* NOP with read bit */

    /* 提取 14 位数据 (去掉 bit14 和 bit15) */
    data &= 0x3FFF;

    return data;
}

/**
 * @brief 读取角度原始值 (带补偿)
 */
static uint16_t as5047_get_angle_raw(void)
{
    return as5047_read_reg(AS5047_REG_ANGLECOM);
}

/**
 * @brief 更新 AS5047P 速度数据
 * @note  处理角度翻转（0->2PI 或 2PI->0），固定采样周期中调用
 * @note  调用频率为10KHz (ADC注入中断)
 * @note  每10次中断累加delta_raw，每1ms(1kHz)计算一次速度
 */
void as5047_update_speed(void)
{
    /* 读取当前角度原始值 (0~16383) */
    uint16_t current_angle_raw = as5047_get_angle_raw();

    /* 首次调用时初始化所有变量 */
    if (!as5047_speed_data.is_initialized)
    {
        as5047_speed_data.last_angle_raw = current_angle_raw;      // 记录初始角度原始值
        as5047_speed_data.last_angle_rad = ((float)current_angle_raw / AS5047_RESOLUTION) * 2.0f * M_PI;  // 记录初始角度(弧度)
        as5047_speed_data.speed_rpm = 0.0f;                        // 初始转速为0
        as5047_speed_data.speed_rpm_lpf = 0.0f;                    // 初始滤波转速为0
        as5047_speed_data.theta_sum = 0;                           // 清零角度增量累加器
        as5047_speed_data.update_cnt = 0;                          // 清零更新计数器
        as5047_speed_data.is_initialized = 1;                      // 标记已初始化
        return;
    }

    /* 计算本次角度增量 delta_raw (当前角度 - 上次角度) */
    int32_t delta_raw = (int32_t)current_angle_raw - (int32_t)as5047_speed_data.last_angle_raw;

    /* 处理角度翻转：当角度增量超过半圈时，判断为跨越0点 */
    /* 例如：从16000跳到100，delta_raw = -15900，实际应该是+484 */
    if (delta_raw > AS5047_RESOLUTION / 2)
    {
        delta_raw -= AS5047_RESOLUTION;  // 正向翻转修正 (0->16383)
    }
    else if (delta_raw < -AS5047_RESOLUTION / 2)
    {
        delta_raw += AS5047_RESOLUTION;  // 反向翻转修正 (16383->0)
    }

    /* theta_sum: 用于速度计算，每1ms清零一次 */
    as5047_speed_data.theta_sum += delta_raw;
    
    /* update_cnt: 更新计数器，每调用一次+1，用于分频(10kHz->1kHz) */
    as5047_speed_data.update_cnt++;

    /* 每10次中断 (1ms/1kHz) 计算一次速度 */
    if (as5047_speed_data.update_cnt >= AS5047_SPEED_CALC_DIV)
    {
        /* 计算转速 (RPM): 
         * speed_rpm = (角度变化量 / 一圈分辨率) * 60秒 / 采样时间
         * 例如：theta_sum=1638, 则转了0.1圈, 在0.001秒内, 速度=0.1*60/0.001=6000RPM
         */
        as5047_speed_data.speed_rpm = ((float)as5047_speed_data.theta_sum / AS5047_RESOLUTION) * 60.0f / AS5047_SPEED_SAMPLE_TIME;

        /* 一阶低通滤波，平滑转速波动，用于显示 */
        as5047_speed_data.speed_rpm_lpf = AS5047_SPEED_FILTER_ALPHA * as5047_speed_data.speed_rpm +
                                          (1.0f - AS5047_SPEED_FILTER_ALPHA) * as5047_speed_data.speed_rpm_lpf;

        /* 清零短期累加器和计数器，准备下一个1ms周期 */
        as5047_speed_data.theta_sum = 0;
        as5047_speed_data.update_cnt = 0;
    }

    /* 更新历史数据，供下次调用使用 */
    as5047_speed_data.last_angle_raw = current_angle_raw;  // 保存当前角度原始值
    as5047_speed_data.last_angle_rad = ((float)current_angle_raw / AS5047_RESOLUTION) * 2.0f * M_PI;  // 保存当前角度(弧度)
}

void as5047_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    /* 使能 CS 引脚时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置 CS 引脚为推挽输出 */
    gpio_init.Pin = GPIO_PIN_15;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* CS 默认拉高 */
    AS5047_CS_HIGH();

    /* 初始化 SPI */
    spi_init();
}

/**
 * @brief 读取电角度 (弧度)
 * @return 电角度 = 机械角度 × 极对数
 */
float as5047_get_angle_rad(void)
{
    uint16_t raw = as5047_get_angle_raw();
    float mech_angle = ((float)raw / (float)AS5047_RESOLUTION) * 2.0f * M_PI;
    return mech_angle * AS5047_MOTOR_POLE_PAIR;
}

/**
 * @brief 读取转速 (RPM) - 未滤波, 用于速度反馈控制
 * @note  返回最近一次更新的速度值，需先调用 as5047_update_speed() 更新速度
 */
float as5047_get_speed_rpm(void)
{
    return as5047_speed_data.speed_rpm;
}

/**
 * @brief 读取滤波后的转速 (RPM) - 用于显示
 * @note  返回最近一次更新的滤波后速度值，需先调用 as5047_update_speed() 更新速度
 */
float as5047_get_speed_rpm_lpf(void)
{
    return as5047_speed_data.speed_rpm_lpf;
}

/**
 * @brief 读取错误标志
 */
uint16_t as5047_get_error(void)
{
    return as5047_read_reg(AS5047_REG_ERRFL);
}
