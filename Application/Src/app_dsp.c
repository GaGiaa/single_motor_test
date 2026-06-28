#include "app_dsp.h"

bool App_DSP_IsCmsisEnabled(void)
{
#if (APP_DSP_ENABLE_CMSIS_DSP != 0U)
    return true;
#else
    return false;
#endif
}

uint32_t App_DSP_GetBuildFlags(void)
{
    uint32_t flags = 0U;

#if (APP_DSP_ENABLE_CMSIS_DSP != 0U)
    flags |= APP_DSP_FLAG_CMSIS_DSP;
#endif
#if defined(ARM_MATH_CM7)
    flags |= APP_DSP_FLAG_ARM_MATH_CM7;
#endif

    return flags;
}

float App_DSP_DotProductF32(const float *a, const float *b, uint32_t length)
{
    float result = 0.0f;

    if (a == 0 || b == 0 || length == 0U) {
        return 0.0f;
    }

#if (APP_DSP_ENABLE_CMSIS_DSP != 0U)
    arm_dot_prod_f32(a, b, length, &result);
#else
    for (uint32_t i = 0U; i < length; ++i) {
        result += a[i] * b[i];
    }
#endif

    return result;
}

void App_DSP_FIRF32(const float *coeffs,
                    uint32_t coeff_count,
                    const float *input,
                    float *output,
                    uint32_t block_size,
                    float *state)
{
    if (coeffs == 0 || input == 0 || output == 0 || state == 0 ||
        coeff_count == 0U || block_size == 0U) {
        return;
    }

#if (APP_DSP_ENABLE_CMSIS_DSP != 0U)
    arm_fir_instance_f32 fir;

    arm_fir_init_f32(&fir, (uint16_t)coeff_count, (float *)coeffs, state, block_size);
    arm_fir_f32(&fir, input, output, block_size);
#else
    for (uint32_t i = 0U; i < (coeff_count + block_size - 1U); ++i) {
        state[i] = 0.0f;
    }
    for (uint32_t n = 0U; n < block_size; ++n) {
        state[n + coeff_count - 1U] = input[n];
        output[n] = 0.0f;
        for (uint32_t k = 0U; k < coeff_count; ++k) {
            output[n] += coeffs[k] * state[n + coeff_count - 1U - k];
        }
    }
#endif
}
