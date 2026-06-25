#ifndef M2006_TASK_H
#define M2006_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "dji_m2006.h"
#include "pid.h"

typedef enum {
    M2006_MODE_STOP = 0,
    M2006_MODE_CURRENT = 1,
    M2006_MODE_SPEED = 2,
    M2006_MODE_POSITION = 3,
} M2006_ControlMode;

typedef struct {
    volatile bool enable;
    volatile M2006_ControlMode mode;

    volatile float target_current_A;
    volatile float target_speed_rpm;
    volatile float target_angle_deg;

    volatile float feedback_speed_rpm;
    volatile float feedback_angle_deg;
    volatile float feedback_total_angle_deg;
    volatile float feedback_current_A;
    volatile uint8_t feedback_temperature_c;

    volatile float command_current_A;
    volatile int16_t command_current_raw;

    volatile uint32_t rx_count;
    volatile uint32_t tx_count;
    volatile uint32_t error_count;

    PID_Param_Config speed_pid_params;
    PID_Param_Config angle_pid_params;
} M2006_Debug;

extern M2006_Debug g_m2006_debug;

void M2006_Task_Run(void *argument);

#endif
