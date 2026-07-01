#ifndef APP_DSP_H
#define APP_DSP_H

#include <stdbool.h>
#include <stdint.h>

#ifndef APP_DSP_ENABLE_CMSIS_DSP
#define APP_DSP_ENABLE_CMSIS_DSP    (0U)
#endif

#define APP_DSP_FLAG_CMSIS_DSP      (1UL << 0)
#define APP_DSP_FLAG_ARM_MATH_CM7   (1UL << 1)

#if (APP_DSP_ENABLE_CMSIS_DSP != 0U)
#include "arm_math.h"
#endif

bool App_DSP_IsCmsisEnabled(void);
uint32_t App_DSP_GetBuildFlags(void);
float App_DSP_DotProductF32(const float *a, const float *b, uint32_t length);
void App_DSP_FIRF32(const float *coeffs,
                    uint32_t coeff_count,
                    const float *input,
                    float *output,
                    uint32_t block_size,
                    float *state);

#endif
