#include "dji_m2006.h"

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

void DJI_M2006_Init(DJI_M2006_State *motor)
{
    if (motor == NULL) {
        return;
    }

    memset(motor, 0, sizeof(*motor));
}

void DJI_M2006_UpdateFeedback(DJI_M2006_State *motor, const uint8_t data[8])
{
    if (motor == NULL || data == NULL) {
        return;
    }

    const uint16_t encoder_raw = (uint16_t)((uint16_t)data[0] << 8) | data[1];
    const int16_t raw_speed_rpm = dji_i16_from_be(data[2], data[3]);
    const int16_t raw_current = dji_i16_from_be(data[4], data[5]);

    if (motor->has_last_encoder) {
        int32_t delta = (int32_t)encoder_raw - (int32_t)motor->last_encoder_raw;
        if (delta > DJI_M2006_ENCODER_HALF) {
            delta -= DJI_M2006_ENCODER_MAX;
        } else if (delta < -DJI_M2006_ENCODER_HALF) {
            delta += DJI_M2006_ENCODER_MAX;
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
                                ((float)DJI_M2006_ENCODER_MAX * DJI_M2006_GEAR_RATIO);
    motor->feedback_total_angle_deg = ((float)motor->total_encoder_delta * 360.0f) /
                                      ((float)DJI_M2006_ENCODER_MAX * DJI_M2006_GEAR_RATIO);
    motor->feedback_speed_rpm = (float)raw_speed_rpm / DJI_M2006_GEAR_RATIO;
    motor->feedback_current_A = DJI_M2006_RawCurrentToA(raw_current);
}

float DJI_M2006_RawCurrentToA(int16_t raw_current)
{
    return ((float)raw_current * DJI_M2006_C610_MAX_CURRENT_A) /
           (float)DJI_M2006_C610_MAX_CURRENT_RAW;
}

int16_t DJI_M2006_CurrentAToRaw(float current_A)
{
    const float limited_current_A = dji_clampf(current_A,
                                               -DJI_M2006_C610_MAX_CURRENT_A,
                                               DJI_M2006_C610_MAX_CURRENT_A);
    const float raw = limited_current_A * (float)DJI_M2006_C610_MAX_CURRENT_RAW /
                      DJI_M2006_C610_MAX_CURRENT_A;

    return (int16_t)(raw >= 0.0f ? raw + 0.5f : raw - 0.5f);
}

void DJI_M2006_PackCurrentFrame(int16_t motor1_current_raw, uint8_t data[8])
{
    if (data == NULL) {
        return;
    }

    memset(data, 0, 8U);
    data[0] = (uint8_t)(((uint16_t)motor1_current_raw >> 8) & 0xFFU);
    data[1] = (uint8_t)((uint16_t)motor1_current_raw & 0xFFU);
}
