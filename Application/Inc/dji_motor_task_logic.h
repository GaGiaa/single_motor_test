#ifndef DJI_MOTOR_TASK_LOGIC_H
#define DJI_MOTOR_TASK_LOGIC_H

#include <stdint.h>

#include "dji_motor_task.h"
#include "pid.h"

#define DJI_MOTOR_POSITION_DIVIDER      (10U)

float DJI_Motor_CalcCommandCurrentStep(DJI_Motor_Debug *debug,
                                       PID_Incremental *speed_pid,
                                       PID_Position *angle_pid,
                                       uint32_t tick_count);

#endif
