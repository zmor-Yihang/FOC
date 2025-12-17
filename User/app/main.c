#include "main.h"

int main(void)
{
    HAL_Init();
    clock_init();
    usart1_init();
    led1_init(); // 初始化LED1
    key_init(); // 初始化按键

    while (1)
    {


    }
}
