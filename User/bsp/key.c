#include "key.h"

/* 按键启停电机 */
void key_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct = {0};

    gpio_init_struct.Pin = GPIO_PIN_4;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    /* 初始化时默认关闭电机驱动 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/* 按键检测函数, 返回0表示按键未按下, 返回1表示按键按下 */
uint8_t key_scan(void)
{
    /* 静态变量 (只初始化一次，后续调用保留上次的值) */
    static uint8_t last_key_state = 0;    /* 记住上一次的电平状态 */
    static uint32_t last_scan_time = 0;   /* 记住上一次扫描的时间 */

    uint8_t current_key_state = 0; /* 当前读到的电平 */
    uint8_t key_down_event = 0;    /* 计算结果 */

    /* 如果距离上次扫描不足 20ms，直接返回, 跳过按键抖动阶段, 实现消抖 */
    if (HAL_GetTick() - last_scan_time < 20)
    {
        return 0;
    }

    /* 更新时间戳，准备干活 */
    last_scan_time = HAL_GetTick();

    /* 读取按键状态 */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
    {
        current_key_state = 1; /* 当前是按下的 */
    }
    else
    {
        current_key_state = 0; /* 当前是松开的 */
    }

    /* key_down_event 只有在 "上次没按(last_key_state=0)" 且 "现在按了(current_key_state=1)" 时才为 1 */
    key_down_event = current_key_state & (current_key_state ^ last_key_state);

    /* 存档当前状态，留给下一次用 */
    last_key_state = current_key_state;

    /* 6. 返回结果 */
    return key_down_event;
}
