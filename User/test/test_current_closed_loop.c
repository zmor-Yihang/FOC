#include "test_current_closed_loop.h"

static uint8_t foc_test_initialized = 0;

void test_current_closed_loop(foc_t *handle, pid_controller_t *pid_id, pid_controller_t *pid_iq)
{
    if (!foc_test_initialized)
    {
        pid_init(pid_id, 0.2f, 0.03f, 0.0f, 12.0f, -12.0f);
        pid_init(pid_iq, 0.2f, 0.03f, 0.0f, 12.0f, -12.0f);
        foc_init(handle, pid_id, pid_iq, NULL);
        foc_set_target(handle, 0.0f, 0.5f, 0.0f);
        foc_test_initialized = 1;
    }

    adc_values_t injected_values;
    float angle_rad = as5047_read_angle_rad();

    angle_rad *= MOTOR_POLE_PAIR;
    adc1_get_injected_values(&injected_values);

    abc_t i_abc = {
        .a = injected_values.ia,
        .b = injected_values.ib,
        .c = injected_values.ic
    };

    alphabeta_t i_alphabeta = clark_transform(i_abc);

    dq_t i_dq_feedback = park_transform(i_alphabeta, angle_rad);

    foc_current_closed_loop(handle, i_dq_feedback, angle_rad);
}