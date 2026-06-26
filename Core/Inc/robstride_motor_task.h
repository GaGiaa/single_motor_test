#ifndef ROBSTRIDE_MOTOR_TASK_H
#define ROBSTRIDE_MOTOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "robstride_motor.h"

typedef struct {
    volatile bool enable;
    volatile bool clear_fault_on_disable;
    volatile bool set_zero_request;

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
    volatile uint32_t last_rx_can_id;
    volatile uint32_t last_tx_can_id;

    volatile uint32_t rx_count;
    volatile uint32_t tx_count;
    volatile uint32_t error_count;
} RobStride_Motor_Debug;

extern RobStride_Motor_Debug g_robstride_motor_debug;

void RobStride_Motor_Task_Run(void *argument);

#endif
