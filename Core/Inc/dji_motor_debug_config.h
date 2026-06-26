#ifndef DJI_MOTOR_DEBUG_CONFIG_H
#define DJI_MOTOR_DEBUG_CONFIG_H

#include "dji_motor.h"

/*
 * Single-motor debug build switches:
 * - DJI_DEBUG_CAN_CHANNEL: 1, 2, or 3
 * - DJI_DEBUG_MOTOR_ID: 1 through 8
 * - DJI_DEBUG_MOTOR_TYPE: DJI_MOTOR_TYPE_M2006 or DJI_MOTOR_TYPE_M3508
 */
#ifndef DJI_DEBUG_CAN_CHANNEL
#define DJI_DEBUG_CAN_CHANNEL      (2U)
#endif

#ifndef DJI_DEBUG_MOTOR_ID
#define DJI_DEBUG_MOTOR_ID         (2U)
#endif

#ifndef DJI_DEBUG_MOTOR_TYPE
#define DJI_DEBUG_MOTOR_TYPE       DJI_MOTOR_TYPE_M2006
#endif

#if (DJI_DEBUG_CAN_CHANNEL < 1U) || (DJI_DEBUG_CAN_CHANNEL > 3U)
#error "DJI_DEBUG_CAN_CHANNEL must be 1, 2, or 3"
#endif

#if (DJI_DEBUG_MOTOR_ID < 1U) || (DJI_DEBUG_MOTOR_ID > 8U)
#error "DJI_DEBUG_MOTOR_ID must be from 1 to 8"
#endif

#if (DJI_DEBUG_MOTOR_TYPE != DJI_MOTOR_TYPE_M2006) && \
    (DJI_DEBUG_MOTOR_TYPE != DJI_MOTOR_TYPE_M3508)
#error "DJI_DEBUG_MOTOR_TYPE must be DJI_MOTOR_TYPE_M2006 or DJI_MOTOR_TYPE_M3508"
#endif

#endif

