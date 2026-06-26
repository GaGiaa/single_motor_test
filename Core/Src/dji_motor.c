#include "dji_motor.h"

#include <stddef.h>
#include <string.h>

static float dji_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t dji_i16_from_be(const uint8_t high, const uint8_t low)
{
    return (int16_t)((uint16_t)((uint16_t)high << 8) | low);
}

DJI_Motor_Config DJI_Motor_MakeConfig(DJI_Motor_Type motor_type, uint8_t motor_id)
{
    DJI_Motor_Config config;
    const uint8_t normalized_id = (motor_id < 1U || motor_id > 8U) ? 1U : motor_id;

    memset(&config, 0, sizeof(config));
    config.motor_type = motor_type;
    config.motor_id = normalized_id;
    config.control_slot = (uint8_t)((normalized_id - 1U) % 4U);
    config.feedback_id = DJI_MOTOR_FEEDBACK_BASE_ID + normalized_id;
    config.control_id = (normalized_id <= 4U) ? DJI_MOTOR_CONTROL_LOW_ID : DJI_MOTOR_CONTROL_HIGH_ID;
    config.max_current_raw = DJI_ESC_CURRENT_MAX_RAW;

    if (motor_type == DJI_MOTOR_TYPE_M3508) {
        config.gear_ratio = DJI_MOTOR_M3508_GEAR_RATIO;
        config.max_current_A = DJI_ESC_C620_MAX_CURRENT_A;
    } else {
        config.motor_type = DJI_MOTOR_TYPE_M2006;
        config.gear_ratio = DJI_MOTOR_M2006_GEAR_RATIO;
        config.max_current_A = DJI_ESC_C610_MAX_CURRENT_A;
    }

    return config;
}

void DJI_Motor_Init(DJI_Motor_State *motor, const DJI_Motor_Config *config)
{
    if (motor == NULL) {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    if (config != NULL) {
        motor->config = *config;
    } else {
        motor->config = DJI_Motor_MakeConfig(DJI_MOTOR_TYPE_M2006, 1U);
    }
}

void DJI_Motor_UpdateFeedback(DJI_Motor_State *motor, const uint8_t data[8])
{
    if (motor == NULL || data == NULL) {
        return;
    }

    const uint16_t encoder_raw = (uint16_t)((uint16_t)data[0] << 8) | data[1];
    const int16_t raw_speed_rpm = dji_i16_from_be(data[2], data[3]);
    const int16_t raw_current = dji_i16_from_be(data[4], data[5]);

    if (motor->has_last_encoder) {
        int32_t delta = (int32_t)encoder_raw - (int32_t)motor->last_encoder_raw;
        if (delta > DJI_MOTOR_ENCODER_HALF) {
            delta -= DJI_MOTOR_ENCODER_MAX;
        } else if (delta < -DJI_MOTOR_ENCODER_HALF) {
            delta += DJI_MOTOR_ENCODER_MAX;
        }
        motor->total_encoder_delta += delta;
    } else {
        motor->has_last_encoder = true;
        motor->total_encoder_delta = (int32_t)encoder_raw;
    }

    motor->last_encoder_raw = encoder_raw;
    motor->encoder_raw = encoder_raw;
    motor->raw_speed_rpm = raw_speed_rpm;
    motor->raw_current = raw_current;
    motor->feedback_temperature_c = data[6];

    motor->feedback_angle_deg = ((float)encoder_raw * 360.0f) /
                                ((float)DJI_MOTOR_ENCODER_MAX * motor->config.gear_ratio);
    motor->feedback_total_angle_deg = ((float)motor->total_encoder_delta * 360.0f) /
                                      ((float)DJI_MOTOR_ENCODER_MAX * motor->config.gear_ratio);
    motor->feedback_speed_rpm = (float)raw_speed_rpm / motor->config.gear_ratio;
    motor->feedback_current_A = DJI_Motor_RawCurrentToA(&motor->config, raw_current);
}

float DJI_Motor_RawCurrentToA(const DJI_Motor_Config *config, int16_t raw_current)
{
    const DJI_Motor_Config default_config = DJI_Motor_MakeConfig(DJI_MOTOR_TYPE_M2006, 1U);
    const DJI_Motor_Config *active_config = (config != NULL) ? config : &default_config;

    return ((float)raw_current * active_config->max_current_A) /
           (float)active_config->max_current_raw;
}

int16_t DJI_Motor_CurrentAToRaw(const DJI_Motor_Config *config, float current_A)
{
    const DJI_Motor_Config default_config = DJI_Motor_MakeConfig(DJI_MOTOR_TYPE_M2006, 1U);
    const DJI_Motor_Config *active_config = (config != NULL) ? config : &default_config;
    const float limited_current_A = dji_clampf(current_A,
                                               -active_config->max_current_A,
                                               active_config->max_current_A);
    const float raw = limited_current_A * (float)active_config->max_current_raw /
                      active_config->max_current_A;

    return (int16_t)(raw >= 0.0f ? raw + 0.5f : raw - 0.5f);
}

void DJI_Motor_PackCurrentFrame(const DJI_Motor_Config *config, int16_t current_raw, uint8_t data[8])
{
    if (data == NULL) {
        return;
    }

    memset(data, 0, 8U);
    if (config == NULL || config->control_slot > 3U) {
        return;
    }

    const uint8_t byte_index = (uint8_t)(config->control_slot * 2U);
    data[byte_index] = (uint8_t)(((uint16_t)current_raw >> 8) & 0xFFU);
    data[byte_index + 1U] = (uint8_t)((uint16_t)current_raw & 0xFFU);
}

