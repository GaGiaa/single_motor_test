#ifndef APP_TIME_PROFILER_H
#define APP_TIME_PROFILER_H

#include <stdbool.h>
#include <stdint.h>

#if !defined(APP_TIME_PROFILER_ENABLE)
#if defined(STM32H723xx)
#define APP_TIME_PROFILER_ENABLE    (1U)
#else
#define APP_TIME_PROFILER_ENABLE    (0U)
#endif
#endif

typedef struct {
    uint32_t rtos_tick;
    uint32_t systick_value;
    uint32_t systick_load;
    bool valid;
} App_TimeProfiler_Timestamp;

typedef struct {
    uint32_t last_us;
    uint32_t max_us;
    uint32_t avg_us;
    uint32_t sample_count;
    bool enabled;
} App_TimeProfiler_Stats;

void App_TimeProfiler_Init(void);
App_TimeProfiler_Timestamp App_TimeProfiler_Begin(void);
void App_TimeProfiler_End(App_TimeProfiler_Stats *stats, App_TimeProfiler_Timestamp start);
void App_TimeProfiler_Reset(App_TimeProfiler_Stats *stats);

#endif
