#include "./utils/delay.h"

void delay_us(uint32_t nus)
{
    uint32_t temp;
    while(nus > 10000)  // 分段处理大延时
    {
        SysTick->LOAD = 170 * 10000;
        SysTick->VAL = 0x00;
        SysTick->CTRL = 0x05;
        do {
            temp = SysTick->CTRL;
        } while ((temp & 0x01) && (!(temp & (1 << 16))));
        nus -= 10000;
    }
    
    if(nus > 0)
    {
        SysTick->LOAD = 170 * nus;
        SysTick->VAL = 0x00;
        SysTick->CTRL = 0x05;
        do {
            temp = SysTick->CTRL;
        } while ((temp & 0x01) && (!(temp & (1 << 16))));
    }
    
    SysTick->CTRL = 0x00;
    SysTick->VAL = 0x00;
}

void delay_ms(uint16_t nms)
{
    while(nms--)
    {
        delay_us(1000);  
    }
}