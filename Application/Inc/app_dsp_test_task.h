#ifndef APP_DSP_TEST_TASK_H
#define APP_DSP_TEST_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifndef APP_DSP_TEST_TASK_ENABLE
#define APP_DSP_TEST_TASK_ENABLE    (1U)
#endif

typedef struct {
    volatile bool enable;
    volatile uint32_t run_count;
    volatile uint32_t pass_count;
    volatile uint32_t fail_count;
    volatile float last_dot_result;
    volatile float expected_dot_result;
    volatile float last_fir_output;
    volatile float expected_fir_output;
    volatile float last_error;
    volatile uint32_t last_run_us;
    volatile uint32_t max_run_us;
    volatile bool dsp_cmsis_enabled;
    volatile uint32_t dsp_build_flags;
} App_DSP_Test_Debug;

extern App_DSP_Test_Debug g_app_dsp_test_debug;

void App_DSP_Test_Task_Run(void *argument);

#endif
