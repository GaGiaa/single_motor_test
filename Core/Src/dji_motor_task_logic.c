#include "dji_motor_task_logic.h"

#include <stddef.h>

float DJI_Motor_CalcCommandCurrentStep(DJI_Motor_Debug *debug,
                                       PID_Incremental *speed_pid,
                                       PID_Position *angle_pid,
                                       uint32_t tick_count)
{
    if (debug == NULL || speed_pid == NULL || angle_pid == NULL) {
        return 0.0f;
    }

    if (!debug->enable || debug->mode == DJI_MOTOR_MODE_STOP) {
        PID_Incremental_Reset(speed_pid);
        PID_Position_Reset(angle_pid);
        return 0.0f;
    }

    if (debug->mode == DJI_MOTOR_MODE_CURRENT) {
        return debug->target_current_A;
    }

    speed_pid->params = debug->speed_pid_params;
    angle_pid->params = debug->angle_pid_params;

    if (debug->mode == DJI_MOTOR_MODE_POSITION &&
        (tick_count % DJI_MOTOR_POSITION_DIVIDER) == 0U) {
        debug->target_speed_rpm = PID_Position_Calc(angle_pid,
                                                    debug->target_angle_deg,
                                                    debug->feedback_total_angle_deg);
    }

    return PID_Incremental_Calc(speed_pid,
                                debug->target_speed_rpm,
                                debug->feedback_speed_rpm);
}
