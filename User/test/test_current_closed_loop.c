#include "test_current_closed_loop.h"

static foc_t foc_handle;
static pid_controller_t pid_id;
static pid_controller_t pid_iq;

void test_current_closed_loop(void)
{
    /* 初始化PID和FOC */
    pid_init(&pid_id, 0.2f, 0.03f, 0.0f, 12.0f, -12.0f);
    pid_init(&pid_iq, 0.2f, 0.03f, 0.0f, 12.0f, -12.0f);
    foc_init(&foc_handle, &pid_id, &pid_iq, NULL);
    foc_set_target(&foc_handle, 0.0f, 0.5f, 0.0f);

    printf("FOC Current Closed Loop Test Started.\n");

    while (1)
    {
        /* 等待ADC注入组转换完成 */
        if (adc_injected_cplt_flag)
        {
            adc_injected_cplt_flag = 0;

            adc_values_t injected_values;
            float angle_rad = as5047_read_angle_rad();

            /* 注意：此处假设机械零点即为电角度零点，实际应用需进行零点对齐 */
            angle_rad *= MOTOR_POLE_PAIR;
            
            adc1_get_injected_values(&injected_values);

            abc_t i_abc = {
                .a = injected_values.ia,
                .b = injected_values.ib,
                .c = injected_values.ic
            };

            alphabeta_t i_alphabeta = clark_transform(i_abc);

            dq_t i_dq_feedback = park_transform(i_alphabeta, angle_rad);

            foc_current_closed_loop(&foc_handle, i_dq_feedback, angle_rad);
        }
        
        /* 检查按键退出循环 */
        if (key_scan() == 1)
        {
            tim1_set_pwm_duty(0, 0, 0); // 停止PWM
            printf("FOC Loop Exit!\r\n");
            break;
        }
    }
}
