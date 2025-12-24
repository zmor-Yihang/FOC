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
    float speed_rpm;           /* 当前转速 (RPM) */
    float speed_rad_s;         /* 当前角速度 (rad/s) */
    int64_t total_angle_count; /* 累计角度计数 (处理圈数) */
    uint32_t last_update_time; /* 上一次更新时间 (ms) */
} as5047_speed_data = {0, 0.0f, 0.0f, 0.0f, 0, 0};

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

    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx_data, (uint8_t *)&rx_data, 1, 1000);

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
 * @brief 读取角度原始值 (带补偿)
 */
static uint16_t as5047_get_angle_raw(void)
{
    return as5047_read_reg(AS5047_REG_ANGLECOM);
}

/**
 * @brief 更新 AS5047P 速度数据
 * @note  处理角度翻转（0->2PI 或 2PI->0）
 */
static void as5047_calculate_speed(void)
{
    // 1. 只读取一次原始值，避免重复采样和数据不一致
    uint16_t current_angle_raw = as5047_get_angle_raw();

    // 2. 获取时间 (如果能换成微秒级更好)
    uint32_t current_time = HAL_GetTick();

    /* 首次初始化处理 */
    if (as5047_speed_data.last_update_time == 0)
    {
        as5047_speed_data.last_angle_raw = current_angle_raw;
        // 本地计算 rad，不要再次通过 SPI 读取
        as5047_speed_data.last_angle_rad = ((float)current_angle_raw / AS5047_RESOLUTION) * 2.0f * M_PI;
        as5047_speed_data.last_update_time = current_time;
        as5047_speed_data.speed_rpm = 0.0f;
        as5047_speed_data.speed_rad_s = 0.0f;
        return;
    }

    /* 计算时间差 */
    float dt = (float)(current_time - as5047_speed_data.last_update_time) / 1000.0f;

    // 简单的防止除零保护，实际应用建议用固定频率调用，不要依赖 dt 计算
    if (dt < 0.001f)
    {
        // 时间间隔太短，无法计算有效速度，直接返回，避免噪声
        return;
    }

    /* --- 核心修正：处理 14位 数据的过零翻转 --- */
    int32_t delta_raw = (int32_t)current_angle_raw - (int32_t)as5047_speed_data.last_angle_raw;

    // AS5047 分辨率为 AS5047_RESOLUTION，半圈为 8192
    if (delta_raw > AS5047_RESOLUTION / 2)
    {
        delta_raw -= AS5047_RESOLUTION; // 判定为反转越过零点
    }
    else if (delta_raw < -AS5047_RESOLUTION / 2)
    {
        delta_raw += AS5047_RESOLUTION; // 判定为正转越过零点
    }

    /* 计算物理角度差 */
    float delta_angle = ((float)delta_raw / AS5047_RESOLUTION) * 2.0f * M_PI;

    /* 累计总角度计数 (用于多圈绝对位置，如果需要) */
    as5047_speed_data.total_angle_count += delta_raw;

    /* 计算原始角速度 (rad/s) */
    float speed_raw = delta_angle / dt;

    /* 一阶低通滤波 */
    as5047_speed_data.speed_rad_s = AS5047_SPEED_FILTER_ALPHA * speed_raw +
                                    (1.0f - AS5047_SPEED_FILTER_ALPHA) * as5047_speed_data.speed_rad_s;

    /* 转换为 RPM */
    as5047_speed_data.speed_rpm = as5047_speed_data.speed_rad_s * 9.549297f; // 60 / 2PI ≈ 9.549

    /* 更新历史状态 */
    as5047_speed_data.last_angle_raw = current_angle_raw;
    // 更新 rad 仅做显示或记录用，不参与下一次核心计算
    as5047_speed_data.last_angle_rad = ((float)current_angle_raw / AS5047_RESOLUTION) * 2.0f * M_PI;
    as5047_speed_data.last_update_time = current_time;
}

/**
 * @brief 读取机械角度 (弧度)
 */
float as5047_get_angle_rad(void)
{
    uint16_t raw = as5047_get_angle_raw();
    return ((float)raw / (float)AS5047_RESOLUTION) * 2.0f * M_PI;
}

/**
 * @brief 读取转速 (RPM)
 */
float as5047_get_speed_rpm(void)
{
    as5047_calculate_speed();
    return as5047_speed_data.speed_rpm;
}

/**
 * @brief 读取错误标志
 */
uint16_t as5047_get_error(void)
{
    return as5047_read_reg(AS5047_REG_ERRFL);
}
