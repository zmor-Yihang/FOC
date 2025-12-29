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

/**
 * @brief 读取角度原始值 (带补偿)
 */
static uint16_t as5047_get_angle_raw(void)
{
    return as5047_read_reg(AS5047_REG_ANGLECOM);
}

/**
 * @brief 更新 AS5047P 速度数据
 * @note  处理角度翻转（0->2PI 或 2PI->0），应在固定采样周期中调用
 * @note  计算频率为10KHz, ADC中断中调用
 */
void as5047_update_speed(void)
{
    uint16_t current_angle_raw = as5047_get_angle_raw();
    uint32_t current_time = HAL_GetTick();

    if (as5047_speed_data.last_update_time == 0)
    {
        as5047_speed_data.last_angle_raw = current_angle_raw;
        as5047_speed_data.last_angle_rad = ((float)current_angle_raw / AS5047_RESOLUTION) * 2.0f * M_PI;
        as5047_speed_data.last_update_time = current_time;
        as5047_speed_data.speed_rpm = 0.0f;
        as5047_speed_data.speed_rad_s = 0.0f;
        return;
    }

    // 中断中调用，固定采样时间
    float dt = AS5047_SPEED_SAMPLE_TIME;

    int32_t delta_raw = (int32_t)current_angle_raw - (int32_t)as5047_speed_data.last_angle_raw;

    // 当角度增量超过半分辨率时，判断为角度翻转
    if (delta_raw > AS5047_RESOLUTION / 2)
    {
        delta_raw -= AS5047_RESOLUTION;
    }
    else if (delta_raw < -AS5047_RESOLUTION / 2)
    {
        delta_raw += AS5047_RESOLUTION;
    }

    float delta_angle = ((float)delta_raw / AS5047_RESOLUTION) * 2.0f * M_PI;

    as5047_speed_data.total_angle_count += delta_raw;

    float speed_raw = delta_angle / dt;

    // 如果计算出的速度与当前速度差异过大，可能是由于噪声或异常值导致
    float speed_diff = fabsf(speed_raw - as5047_speed_data.speed_rad_s);
    float max_reasonable_speed = 400.0f; // 提高最大合理速度限制，可根据实际应用调整

    if (speed_diff < max_reasonable_speed)
    {
        // 正常滤波
        as5047_speed_data.speed_rad_s = AS5047_SPEED_FILTER_ALPHA * speed_raw +
                                        (1.0f - AS5047_SPEED_FILTER_ALPHA) * as5047_speed_data.speed_rad_s;
    }
    else
    {
        // 如果速度变化过大，降低滤波强度以更快响应
        as5047_speed_data.speed_rad_s = 0.3f * speed_raw + 0.7f * as5047_speed_data.speed_rad_s;
    }

    as5047_speed_data.speed_rpm = as5047_speed_data.speed_rad_s * 9.549297f;
    as5047_speed_data.last_angle_raw = current_angle_raw;
    as5047_speed_data.last_angle_rad = ((float)current_angle_raw / AS5047_RESOLUTION) * 2.0f * M_PI;
    as5047_speed_data.last_update_time = current_time;
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
 * @brief 读取机械角度 (弧度)
 */
float as5047_get_angle_rad(void)
{
    uint16_t raw = as5047_get_angle_raw();
    return ((float)raw / (float)AS5047_RESOLUTION) * 2.0f * M_PI;
}

/**
 * @brief 读取转速 (RPM)
 * @note  返回最近一次更新的速度值，需先调用 as5047_update_speed() 更新速度
 */
float as5047_get_speed_rpm(void)
{
    return as5047_speed_data.speed_rpm;
}

/**
 * @brief 读取错误标志
 */
uint16_t as5047_get_error(void)
{
    return as5047_read_reg(AS5047_REG_ERRFL);
}
