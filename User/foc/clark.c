#include "clark.h"

void clark_transform(phase3_t input, phase2_t *output)
{
    float one_by_sqrt3 = 1.0f / sqrtf(3.0f);

    // iα = Ia
    output->axis_1 = input.axis_1;

    // iβ = (1/√3)Ia + (2/√3)Ib
    output->axis_2 = one_by_sqrt3 * input.axis_1 + 2.0f * one_by_sqrt3 * input.axis_2;
}

void iclark_transform(phase2_t input, phase3_t *output)
{
    float sqrt3_by_2 = sqrtf(3.0f) / 2.0f;

    // Ia = iα
    output->axis_1 = input.axis_1;

    // Ib = -iα/2 + (√3/2)iβ
    output->axis_2 = -0.5f * input.axis_1 + sqrt3_by_2 * input.axis_2;

    // Ic = -iα/2 - (√3/2)iβ
    output->axis_3 = -output->axis_1 - output->axis_2;
}