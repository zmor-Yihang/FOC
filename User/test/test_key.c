#include "test_key.h"


void test_key(void)
{
    // 检测按键是否被按下
    if(key_scan())
    {
        // 按键按下时翻转LED状态
        led1_toggle();

    }
}