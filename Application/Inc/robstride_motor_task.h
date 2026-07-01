#ifndef ROBSTRIDE_MOTOR_TASK_H
#define ROBSTRIDE_MOTOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "robstride_motor.h"

typedef struct {
    volatile bool request;
    volatile uint32_t branch_count;
    volatile uint32_t request_count;
    volatile uint32_t success_count;
    volatile uint32_t send_fail_count;
    volatile uint32_t unhandled_count;
    volatile uint32_t fail_count;
    volatile uint8_t last_fail_status;
    volatile uint8_t last_u8_value;
    volatile uint16_t param_index;
    volatile uint8_t param_target;
    volatile uint32_t last_tx_can_id;
    volatile uint32_t last_success_rx_can_id;
    volatile uint32_t last_fail_rx_can_id;
    volatile bool last_u8_value_valid;
} RobStride_Motor_Type17_Read_Debug;

typedef struct {
    volatile bool enable;
    volatile bool clear_fault_on_disable;
    volatile bool set_zero_request;
    volatile bool request_device_id;
    volatile bool request_active_report;
    volatile bool active_report_enable;

    volatile float target_position_rad;
    volatile float target_velocity_rad_s;
    volatile float target_torque_Nm;
    volatile float kp;
    volatile float kd;

    volatile float feedback_position_rad;
    volatile float feedback_velocity_rad_s;
    volatile float feedback_torque_Nm;
    volatile float feedback_temperature_c;
    volatile uint8_t mode_state;
    volatile uint8_t fault_code;

    volatile uint8_t configured_can_channel;
    volatile uint8_t configured_motor_id;
    volatile uint8_t configured_host_can_id;
    volatile RobStride_Motor_Type configured_motor_type;
    volatile uint32_t device_id_request_count;
    volatile uint32_t device_id_response_count;
    volatile uint32_t active_report_request_count;
    volatile uint8_t last_discovered_motor_id;
    volatile uint8_t last_device_uid[8];
    volatile uint32_t last_device_id_rx_can_id;
    volatile bool device_id_valid;
    volatile RobStride_Motor_Type17_Read_Debug type17_read;
    volatile uint32_t raw_rx_count;
    volatile uint32_t last_unhandled_rx_can_id;
    volatile uint8_t last_unhandled_rx_data[8];
    volatile uint32_t last_rx_can_id;
    volatile uint32_t last_tx_can_id;

    volatile uint32_t rx_count;
    volatile uint32_t tx_count;
    volatile uint32_t error_count;

    volatile bool dsp_cmsis_enabled;
    volatile uint32_t dsp_build_flags;
    volatile bool time_profiler_enabled;
    volatile uint32_t last_loop_us;
    volatile uint32_t max_loop_us;
    volatile uint32_t avg_loop_us;
} RobStride_Motor_Debug;

extern RobStride_Motor_Debug g_robstride_motor_debug;

void RobStride_Motor_Task_Run(void *argument);

#endif
