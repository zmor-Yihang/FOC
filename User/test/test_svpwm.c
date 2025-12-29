#include "test_svpwm.h"
#include "../foc/clark_park.h"

/**
 * @brief  SVPWM 测试函数
 * @note   测试 SVPWM 算法输入/输出关系，以及占空比限幅
 */
void test_svpwm(void)
{
    float t = 0.0f;
    float dt = 0.00001f;    // 采样周期 1ms
    float freq = 50.0f;     // 频率 50Hz
    float amplitude = 0.8f; // 调制电压幅值 (归一化，范围 [-1, 1])

    abc_t u_abc, duty;
    float phase_a, phase_b, phase_c;

    while (1)
    {
        // 1. 生成三相正弦波信号 (120°相位差)
        //    为了验证 min-max 零序注入，使用对称的三相波形
        phase_a = 2.0f * 3.1415926f * freq * t;
        phase_b = phase_a - 2.0f * 3.1415926f / 3.0f; // 滞后 120°
        phase_c = phase_a + 2.0f * 3.1415926f / 3.0f; // 超前 120°

        u_abc.a = amplitude * fast_sin(phase_a);
        u_abc.b = amplitude * fast_sin(phase_b);
        u_abc.c = amplitude * fast_sin(phase_c);

        // 2. 调用 SVPWM 算法
        alphabeta_t u_ab = clark_transform(u_abc);
        duty = svpwm_update(u_ab);

        // 3. 检查占空比是否在合理范围内
        //    占空比应该在 [0, 1] 之间
        if (duty.a < 0.0f || duty.a > 1.0f ||
            duty.b < 0.0f || duty.b > 1.0f ||
            duty.c < 0.0f || duty.c > 1.0f)
        {
            printf("ERROR: Duty out of range! a=%.3f, b=%.3f, c=%.3f\r\n",
                       duty.a, duty.b, duty.c);
        }

        // 4. 通过 VOFA 打印输出结果用于波形显示
        //    格式: u_a, u_b, u_c, duty_a, duty_b, duty_c, 校验值(u_a+u_b+u_c)
        printf("%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n",
                   u_abc.a, u_abc.b, u_abc.c,
                   duty.a, duty.b, duty.c,
                   u_abc.a + u_abc.b + u_abc.c);

        HAL_Delay(10); // 延迟 10ms，便于观察
        t += dt * 10;  // 实际时间增量

        if (key_scan() == 1)
        {
            printf("SVPWM Test Stop!\r\n");
            return;
        }
    }
}
