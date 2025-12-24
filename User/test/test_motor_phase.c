#include "test_motor_phase.h"

void test_raw_six_step(void)
{
    printf("======= 六步换相原始测试 =======\n");
    printf("完全绕过FOC算法，直接设置PWM\n\n");
    
    float high = 0.7f;
    float low = 0.3f;
    
    // 换相序列
    typedef struct {
        float a, b, c;
        char name[10];
    } Step;
    
    Step steps[] = {
        {high, low,  low,  "A+B-"},
        {high, low,  high, "A+C-"},
        {low,  low,  high, "B+C-"},
        {low,  high, high, "B+A-"},
        {low,  high, low,  "C+A-"},
        {high, high, low,  "C+B-"},
    };
    
    for (int cycle = 0; cycle < 5; cycle++) {
        printf("\n循环 %d:\n", cycle + 1);
        for (int i = 0; i < 6; i++) {
            printf("步骤 %d: %s -> (%.1f, %.1f, %.1f)\n", 
                   i+1, steps[i].name, steps[i].a, steps[i].b, steps[i].c);
            
            tim1_set_pwm_duty(steps[i].a, steps[i].b, steps[i].c);
            HAL_Delay(500);  // 每步停留500ms，应该能看到明显的转动
        }
    }
    
    tim1_set_pwm_duty(0, 0, 0);
    printf("\n测试完成\n");
    printf("如果电机转了：硬件正常，问题在FOC算法\n");
    printf("如果电机不转：硬件或驱动板有问题\n");
}