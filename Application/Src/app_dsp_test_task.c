#include "app_dsp_test_task.h"

#include "app_dsp.h"
#include "app_time_profiler.h"
#include "cmsis_os.h"

#include <stddef.h>

#define APP_DSP_TEST_PERIOD_TICKS       (100U)
#define APP_DSP_TEST_TOLERANCE          (0.0001f)
#define APP_DSP_TEST_FIR_TAP_COUNT      (4U)
#define APP_DSP_TEST_FIR_BLOCK_SIZE     (4U)

App_DSP_Test_Debug g_app_dsp_test_debug = {
    .enable = true,
    .run_count = 0U,
    .pass_count = 0U,
    .fail_count = 0U,
    .last_dot_result = 0.0f,
    .expected_dot_result = 70.0f,
    .last_fir_output = 0.0f,
    .expected_fir_output = 2.5f,
    .last_error = 0.0f,
    .last_run_us = 0U,
    .max_run_us = 0U,
    .dsp_cmsis_enabled = false,
    .dsp_build_flags = 0U,
};

static float App_DSP_Test_AbsF(float value)
{
    return value < 0.0f ? -value : value;
}

static void App_DSP_Test_UpdateBuildInfo(void)
{
    g_app_dsp_test_debug.dsp_cmsis_enabled = App_DSP_IsCmsisEnabled();
    g_app_dsp_test_debug.dsp_build_flags = App_DSP_GetBuildFlags();
}

static bool App_DSP_Test_RunOnce(uint32_t *elapsed_us)
{
    static const float dot_a[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    static const float dot_b[4] = {5.0f, 6.0f, 7.0f, 8.0f};
    static const float fir_coeffs[APP_DSP_TEST_FIR_TAP_COUNT] = {0.25f, 0.25f, 0.25f, 0.25f};
    static const float fir_input[APP_DSP_TEST_FIR_BLOCK_SIZE] = {1.0f, 2.0f, 3.0f, 4.0f};
    float fir_output[APP_DSP_TEST_FIR_BLOCK_SIZE] = {0.0f};
    float fir_state[APP_DSP_TEST_FIR_TAP_COUNT + APP_DSP_TEST_FIR_BLOCK_SIZE - 1U] = {0.0f};
    App_TimeProfiler_Stats time_stats;
    App_TimeProfiler_Timestamp start;
    const float expected_dot = 70.0f;
    const float expected_fir = 2.5f;
    float dot_error;
    float fir_error;

    App_TimeProfiler_Reset(&time_stats);
    start = App_TimeProfiler_Begin();

    g_app_dsp_test_debug.last_dot_result = App_DSP_DotProductF32(dot_a, dot_b, 4U);
    App_DSP_FIRF32(fir_coeffs,
                   APP_DSP_TEST_FIR_TAP_COUNT,
                   fir_input,
                   fir_output,
                   APP_DSP_TEST_FIR_BLOCK_SIZE,
                   fir_state);
    g_app_dsp_test_debug.last_fir_output = fir_output[APP_DSP_TEST_FIR_BLOCK_SIZE - 1U];

    App_TimeProfiler_End(&time_stats, start);
    if (elapsed_us != NULL) {
        *elapsed_us = time_stats.last_us;
    }

    g_app_dsp_test_debug.expected_dot_result = expected_dot;
    g_app_dsp_test_debug.expected_fir_output = expected_fir;

    dot_error = App_DSP_Test_AbsF(g_app_dsp_test_debug.last_dot_result - expected_dot);
    fir_error = App_DSP_Test_AbsF(g_app_dsp_test_debug.last_fir_output - expected_fir);
    g_app_dsp_test_debug.last_error = dot_error > fir_error ? dot_error : fir_error;

    return dot_error <= APP_DSP_TEST_TOLERANCE && fir_error <= APP_DSP_TEST_TOLERANCE;
}

void App_DSP_Test_Task_Run(void *argument)
{
    uint32_t wake_tick;

    (void)argument;

    App_TimeProfiler_Init();
    App_DSP_Test_UpdateBuildInfo();
    wake_tick = osKernelGetTickCount();

    for (;;) {
        if (g_app_dsp_test_debug.enable) {
            uint32_t elapsed_us = 0U;

            if (App_DSP_Test_RunOnce(&elapsed_us)) {
                ++g_app_dsp_test_debug.pass_count;
            } else {
                ++g_app_dsp_test_debug.fail_count;
            }

            ++g_app_dsp_test_debug.run_count;
            g_app_dsp_test_debug.last_run_us = elapsed_us;
            if (elapsed_us > g_app_dsp_test_debug.max_run_us) {
                g_app_dsp_test_debug.max_run_us = elapsed_us;
            }
            App_DSP_Test_UpdateBuildInfo();
        }

        osDelayUntil(wake_tick + APP_DSP_TEST_PERIOD_TICKS);
        wake_tick += APP_DSP_TEST_PERIOD_TICKS;
    }
}
