#ifndef DJI_M2006_H
#define DJI_M2006_H

#include <stdbool.h>
#include <stdint.h>

#define DJI_M2006_FEEDBACK_ID            (0x201U)
#define DJI_M2006_CONTROL_ID             (0x200U)
#define DJI_M2006_ENCODER_MAX            (8192)
#define DJI_M2006_ENCODER_HALF           (4096)
#define DJI_M2006_GEAR_RATIO             (36.0f)
#define DJI_M2006_C610_MAX_CURRENT_A     (10.0f)
#define DJI_M2006_C610_MAX_CURRENT_RAW   (16384)

typedef struct {
    uint16_t encoder_raw;
    uint16_t last_encoder_raw;
    int32_t total_encoder_delta;
    bool has_last_encoder;
    int16_t raw_speed_rpm;
    int16_t raw_current;
    uint8_t feedback_temperature_c;
    float feedback_angle_deg;
    float feedback_total_angle_deg;
    float feedback_speed_rpm;
    float feedback_current_A;
} DJI_M2006_State;

void DJI_M2006_Init(DJI_M2006_State *motor);
void DJI_M2006_UpdateFeedback(DJI_M2006_State *motor, const uint8_t data[8]);
float DJI_M2006_RawCurrentToA(int16_t raw_current);
int16_t DJI_M2006_CurrentAToRaw(float current_A);
void DJI_M2006_PackCurrentFrame(int16_t motor1_current_raw, uint8_t data[8]);

#endif
