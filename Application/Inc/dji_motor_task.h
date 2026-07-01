#ifndef DJI_MOTOR_TASK_H
#define DJI_MOTOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "dji_motor.h"
#include "pid.h"

typedef enum {
    DJI_MOTOR_MODE_STOP = 0,
    DJI_MOTOR_MODE_CURRENT = 1,
    DJI_MOTOR_MODE_SPEED = 2,
    DJI_MOTOR_MODE_POSITION = 3,
} DJI_Motor_ControlMode;

typedef struct {
    volatile bool enable;
    volatile DJI_Motor_ControlMode mode;

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

    volatile uint8_t configured_can_channel;
    volatile uint8_t configured_motor_id;
    volatile DJI_Motor_Type configured_motor_type;
    volatile uint32_t feedback_can_id;
    volatile uint32_t control_can_id;
    volatile uint8_t control_slot;

    volatile uint32_t rx_count;
    volatile uint32_t tx_count;
    volatile uint32_t error_count;

    volatile bool dsp_cmsis_enabled;
    volatile uint32_t dsp_build_flags;
    volatile bool time_profiler_enabled;
    volatile uint32_t last_loop_us;
    volatile uint32_t max_loop_us;
    volatile uint32_t avg_loop_us;

    PID_Incremental_Param_Config speed_pid_params;
    PID_Position_Param_Config angle_pid_params;
} DJI_Motor_Debug;

extern DJI_Motor_Debug g_dji_motor_debug;

void DJI_Motor_Task_Run(void *argument);

#endif


