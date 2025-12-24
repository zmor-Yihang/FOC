#include "test_as5047.h"

/* AS5047P 测试标志 */
static uint8_t as5047_initialized = 0;

/**
 * @brief AS5047P 编码器测试函数
 */
void test_as5047(void)
{
    static uint32_t last_tick = 0;
    uint32_t current_tick;

    /* 首次调用时初始化 */
    if (!as5047_initialized)
    {
        as5047_init();
        as5047_initialized = 1;

        vofa_print(&huart1, "=== AS5047P Encoder Test Start ===\r\n");
        vofa_print(&huart1, "Resolution: 14-bit (%d), Testing...\r\n\r\n", AS5047_RESOLUTION);
    }

    /* 每次循环更新速度数据 */
    /* 速度会在调用 get_speed_rpm() 时自动更新 */

    /* 每 100ms 打印一次数据 */
    current_tick = HAL_GetTick();
    if (current_tick - last_tick >= 100)
    {
        last_tick = current_tick;

        /* 读取角度信息 */
        /* uint16_t raw_angle = as5047_read_angle_raw();  <-- 现在是内部函数，无法外部调用 */
        float angle_rad = as5047_get_angle_rad();

        /* 读取速度信息 */
        float speed_rpm = as5047_get_speed_rpm();
        uint16_t error = as5047_get_error();

        /* 打印角度数据 */
        /* vofa_print(&huart1, "Angle Raw: %5u | Rad: %.4f\r\n", raw_angle, angle_rad); */
        vofa_print(&huart1, "Angle Rad: %.4f\r\n", angle_rad);

        /* 打印速度数据 */
        vofa_print(&huart1, "Speed: %.1f RPM\r\n",
                   speed_rpm);

        /* 打印诊断信息 */
        vofa_print(&huart1, "Error: 0x%04X\r\n",
                   error);

        /* 检查错误 */
        if (error != 0)
        {
            vofa_print(&huart1, "WARNING: AS5047P Error detected! (0x%04X)\r\n", error);
        }

        vofa_print(&huart1, "\r\n");
    }

    /* 延时 1ms */
    HAL_Delay(1);
}
