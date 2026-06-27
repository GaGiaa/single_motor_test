#include "robstride_motor.h"
#include "app_math.h"

#include <stddef.h>
#include <string.h>

static uint16_t robstride_be_u16(const uint8_t data[8], uint8_t index)
{
    return (uint16_t)(((uint16_t)data[index] << 8) | data[index + 1U]);
}

static void robstride_put_be_u16(uint8_t data[8], uint8_t index, uint16_t value)
{
    data[index] = (uint8_t)(value >> 8);
    data[index + 1U] = (uint8_t)(value & 0xFFU);
}

uint16_t RobStride_Motor_FloatToUint16(float value, float min_value, float max_value)
{
    float clamped = App_Math_ClampFloat(value, min_value, max_value);

    if (max_value <= min_value) {
        return 0U;
    }

    return (uint16_t)((clamped - min_value) * 65535.0f / (max_value - min_value));
}

float RobStride_Motor_Uint16ToFloat(uint16_t value, float min_value, float max_value)
{
    return ((float)value * (max_value - min_value) / 65535.0f) + min_value;
}

uint32_t RobStride_Motor_BuildCanId(RobStride_CommType comm_type, uint16_t data_area2, uint8_t target_id)
{
    return (((uint32_t)comm_type & 0x1FU) << 24) |
           (((uint32_t)data_area2 & 0xFFFFU) << 8) |
           ((uint32_t)target_id & 0xFFU);
}

uint8_t RobStride_Motor_GetCommType(uint32_t can_id)
{
    return (uint8_t)((can_id & ROBSTRIDE_CAN_EXT_ID_MASK) >> 24);
}

uint16_t RobStride_Motor_GetDataArea2(uint32_t can_id)
{
    return (uint16_t)(((can_id & ROBSTRIDE_CAN_EXT_ID_MASK) >> 8) & 0xFFFFU);
}

uint8_t RobStride_Motor_GetTargetId(uint32_t can_id)
{
    return (uint8_t)(can_id & 0xFFU);
}

RobStride_Motor_Config RobStride_Motor_MakeConfig(RobStride_Motor_Type motor_type,
                                                  uint8_t motor_id,
                                                  uint8_t host_can_id)
{
    RobStride_Motor_Config config;

    memset(&config, 0, sizeof(config));
    config.motor_type = ROBSTRIDE_MOTOR_TYPE_EL05;
    config.motor_id = (motor_id == 0U || motor_id > 16U) ? 1U : motor_id;
    config.host_can_id = host_can_id;

    (void)motor_type;
    config.params.position_min_rad = ROBSTRIDE_EL05_POSITION_MIN_RAD;
    config.params.position_max_rad = ROBSTRIDE_EL05_POSITION_MAX_RAD;
    config.params.velocity_min_rad_s = ROBSTRIDE_EL05_VELOCITY_MIN_RAD_S;
    config.params.velocity_max_rad_s = ROBSTRIDE_EL05_VELOCITY_MAX_RAD_S;
    config.params.torque_min_Nm = ROBSTRIDE_EL05_TORQUE_MIN_NM;
    config.params.torque_max_Nm = ROBSTRIDE_EL05_TORQUE_MAX_NM;
    config.params.kp_min = ROBSTRIDE_EL05_KP_MIN;
    config.params.kp_max = ROBSTRIDE_EL05_KP_MAX;
    config.params.kd_min = ROBSTRIDE_EL05_KD_MIN;
    config.params.kd_max = ROBSTRIDE_EL05_KD_MAX;

    return config;
}

void RobStride_Motor_Init(RobStride_Motor_State *motor, const RobStride_Motor_Config *config)
{
    if (motor == NULL) {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->config = (config != NULL) ? *config : RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 1U, 0xFDU);
    motor->motor_id = motor->config.motor_id;
}

void RobStride_Motor_MakeMotionControlFrame(const RobStride_Motor_Config *config,
                                            const RobStride_Motor_Command *command,
                                            RobStride_Motor_Frame *frame)
{
    const RobStride_Motor_Config default_config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 1U, 0xFDU);
    const RobStride_Motor_Command zero_command = {0};
    const RobStride_Motor_Config *active_config = (config != NULL) ? config : &default_config;
    const RobStride_Motor_Command *active_command = (command != NULL) ? command : &zero_command;
    const RobStride_Motor_Params *params = &active_config->params;
    uint16_t torque_raw;

    if (frame == NULL) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    torque_raw = RobStride_Motor_FloatToUint16(active_command->torque_Nm,
                                               params->torque_min_Nm,
                                               params->torque_max_Nm);
    frame->can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_MOTION_CONTROL,
                                               torque_raw,
                                               active_config->motor_id);
    robstride_put_be_u16(frame->data, 0U, RobStride_Motor_FloatToUint16(active_command->position_rad,
                                                                        params->position_min_rad,
                                                                        params->position_max_rad));
    robstride_put_be_u16(frame->data, 2U, RobStride_Motor_FloatToUint16(active_command->velocity_rad_s,
                                                                        params->velocity_min_rad_s,
                                                                        params->velocity_max_rad_s));
    robstride_put_be_u16(frame->data, 4U, RobStride_Motor_FloatToUint16(active_command->kp,
                                                                        params->kp_min,
                                                                        params->kp_max));
    robstride_put_be_u16(frame->data, 6U, RobStride_Motor_FloatToUint16(active_command->kd,
                                                                        params->kd_min,
                                                                        params->kd_max));
}

void RobStride_Motor_MakeEnableFrame(const RobStride_Motor_Config *config, RobStride_Motor_Frame *frame)
{
    const RobStride_Motor_Config default_config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 1U, 0xFDU);
    const RobStride_Motor_Config *active_config = (config != NULL) ? config : &default_config;

    if (frame == NULL) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_ENABLE,
                                               active_config->host_can_id,
                                               active_config->motor_id);
}

void RobStride_Motor_MakeDisableFrame(const RobStride_Motor_Config *config,
                                      bool clear_fault,
                                      RobStride_Motor_Frame *frame)
{
    const RobStride_Motor_Config default_config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 1U, 0xFDU);
    const RobStride_Motor_Config *active_config = (config != NULL) ? config : &default_config;

    if (frame == NULL) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_DISABLE,
                                               active_config->host_can_id,
                                               active_config->motor_id);
    frame->data[0] = clear_fault ? 1U : 0U;
}

void RobStride_Motor_MakeSetZeroFrame(const RobStride_Motor_Config *config, RobStride_Motor_Frame *frame)
{
    const RobStride_Motor_Config default_config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 1U, 0xFDU);
    const RobStride_Motor_Config *active_config = (config != NULL) ? config : &default_config;

    if (frame == NULL) {
        return;
    }

    memset(frame, 0, sizeof(*frame));
    frame->can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_SET_ZERO,
                                               active_config->host_can_id,
                                               active_config->motor_id);
    frame->data[0] = 1U;
}

bool RobStride_Motor_UpdateFeedback(RobStride_Motor_State *motor,
                                    uint32_t can_id,
                                    const uint8_t data[8])
{
    uint8_t motor_id;
    uint8_t comm_type;

    if (motor == NULL || data == NULL) {
        return false;
    }

    comm_type = RobStride_Motor_GetCommType(can_id);
    motor_id = (uint8_t)((can_id >> 8) & 0xFFU);
    if (comm_type != ROBSTRIDE_COMM_FEEDBACK || motor_id != motor->config.motor_id) {
        return false;
    }

    motor->motor_id = motor_id;
    motor->fault_code = (uint8_t)((can_id >> 16) & 0x3FU);
    motor->mode_state = (uint8_t)((can_id >> 22) & 0x03U);
    motor->position_rad = RobStride_Motor_Uint16ToFloat(robstride_be_u16(data, 0U),
                                                        motor->config.params.position_min_rad,
                                                        motor->config.params.position_max_rad);
    motor->velocity_rad_s = RobStride_Motor_Uint16ToFloat(robstride_be_u16(data, 2U),
                                                          motor->config.params.velocity_min_rad_s,
                                                          motor->config.params.velocity_max_rad_s);
    motor->torque_Nm = RobStride_Motor_Uint16ToFloat(robstride_be_u16(data, 4U),
                                                     motor->config.params.torque_min_Nm,
                                                     motor->config.params.torque_max_Nm);
    motor->temperature_c = (float)robstride_be_u16(data, 6U) / 10.0f;
    motor->is_valid = true;

    return true;
}
