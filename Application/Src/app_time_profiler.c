#include "app_time_profiler.h"

#if (APP_TIME_PROFILER_ENABLE != 0U)
#include "cmsis_os.h"
#include "stm32h7xx.h"
#endif

void App_TimeProfiler_Init(void)
{
}

#if (APP_TIME_PROFILER_ENABLE != 0U)
static bool App_TimeProfiler_IsRunning(void)
{
    return osKernelGetState() == osKernelRunning;
}

static uint32_t App_TimeProfiler_TickPeriodUs(void)
{
    return (uint32_t)(1000000UL / (uint32_t)configTICK_RATE_HZ);
}

static App_TimeProfiler_Timestamp App_TimeProfiler_ReadTimestamp(void)
{
    App_TimeProfiler_Timestamp timestamp = {0U, 0U, 0U, false};
    uint32_t tick_before;
    uint32_t tick_after;

    if (!App_TimeProfiler_IsRunning() || SysTick->LOAD == 0U) {
        return timestamp;
    }

    tick_before = osKernelGetTickCount();
    timestamp.systick_value = SysTick->VAL;
    tick_after = osKernelGetTickCount();
    if (tick_after != tick_before) {
        tick_before = tick_after;
        timestamp.systick_value = SysTick->VAL;
    }

    timestamp.rtos_tick = tick_before;
    timestamp.systick_load = SysTick->LOAD;
    timestamp.valid = true;
    return timestamp;
}

static uint64_t App_TimeProfiler_ToUs(App_TimeProfiler_Timestamp timestamp)
{
    const uint32_t reload_counts = timestamp.systick_load + 1U;
    const uint32_t elapsed_counts = reload_counts - timestamp.systick_value;
    const uint64_t tick_us = App_TimeProfiler_TickPeriodUs();
    const uint64_t subtick_us = ((uint64_t)elapsed_counts * tick_us) / reload_counts;

    return ((uint64_t)timestamp.rtos_tick * tick_us) + subtick_us;
}
#endif

App_TimeProfiler_Timestamp App_TimeProfiler_Begin(void)
{
#if (APP_TIME_PROFILER_ENABLE != 0U)
    return App_TimeProfiler_ReadTimestamp();
#else
    App_TimeProfiler_Timestamp timestamp = {0U, 0U, 0U, false};
    return timestamp;
#endif
}

void App_TimeProfiler_End(App_TimeProfiler_Stats *stats, App_TimeProfiler_Timestamp start)
{
    uint32_t elapsed_us = 0U;

    if (stats == NULL) {
        return;
    }

#if (APP_TIME_PROFILER_ENABLE != 0U)
    const App_TimeProfiler_Timestamp end = App_TimeProfiler_ReadTimestamp();
    if (start.valid && end.valid) {
        const uint64_t start_us = App_TimeProfiler_ToUs(start);
        const uint64_t end_us = App_TimeProfiler_ToUs(end);
        elapsed_us = end_us >= start_us ? (uint32_t)(end_us - start_us) : 0U;
        stats->enabled = true;
    } else {
        stats->enabled = false;
    }
#else
    (void)start;
    stats->enabled = false;
#endif

    stats->last_us = elapsed_us;
    if (elapsed_us > stats->max_us) {
        stats->max_us = elapsed_us;
    }
    if (stats->sample_count == 0U) {
        stats->avg_us = elapsed_us;
    } else {
        stats->avg_us = (uint32_t)(((uint64_t)stats->avg_us * stats->sample_count + elapsed_us) /
                                   (stats->sample_count + 1U));
    }
    ++stats->sample_count;
}

void App_TimeProfiler_Reset(App_TimeProfiler_Stats *stats)
{
    if (stats == NULL) {
        return;
    }

    stats->last_us = 0U;
    stats->max_us = 0U;
    stats->avg_us = 0U;
    stats->sample_count = 0U;
#if (APP_TIME_PROFILER_ENABLE != 0U)
    stats->enabled = App_TimeProfiler_IsRunning();
#else
    stats->enabled = false;
#endif
}
