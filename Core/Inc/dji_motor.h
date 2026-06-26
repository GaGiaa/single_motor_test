#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define DJI_MOTOR_FEEDBACK_BASE_ID       (0x200U)
#define DJI_MOTOR_CONTROL_LOW_ID         (0x200U)
#define DJI_MOTOR_CONTROL_HIGH_ID        (0x1FFU)
#define DJI_MOTOR_ENCODER_MAX            (8192)
#define DJI_MOTOR_ENCODER_HALF           (4096)
#define DJI_MOTOR_M2006_GEAR_RATIO      (36.0f)
#define DJI_ESC_C610_MAX_CURRENT_A       (10.0f)
#define DJI_ESC_C620_MAX_CURRENT_A       (20.0f)
#define DJI_ESC_CURRENT_MAX_RAW          (16384)

/* RoboMaster M3508 P19 specifies an exact reduction ratio of 3591/187. */
#define DJI_MOTOR_M3508_GEAR_RATIO       (3591.0f / 187.0f)

typedef enum {
    DJI_MOTOR_TYPE_M2006 = 0,
    DJI_MOTOR_TYPE_M3508 = 1,
} DJI_Motor_Type;

typedef struct {
    DJI_Motor_Type motor_type;
    uint8_t motor_id;
    uint8_t control_slot;
    uint32_t feedback_id;
    uint32_t control_id;
    float gear_ratio;
    float max_current_A;
    int16_t max_current_raw;
} DJI_Motor_Config;

typedef struct {
    DJI_Motor_Config config;
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
} DJI_Motor_State;

DJI_Motor_Config DJI_Motor_MakeConfig(DJI_Motor_Type motor_type, uint8_t motor_id);
void DJI_Motor_Init(DJI_Motor_State *motor, const DJI_Motor_Config *config);
void DJI_Motor_UpdateFeedback(DJI_Motor_State *motor, const uint8_t data[8]);
float DJI_Motor_RawCurrentToA(const DJI_Motor_Config *config, int16_t raw_current);
int16_t DJI_Motor_CurrentAToRaw(const DJI_Motor_Config *config, float current_A);
void DJI_Motor_PackCurrentFrame(const DJI_Motor_Config *config, int16_t current_raw, uint8_t data[8]);

#endif

