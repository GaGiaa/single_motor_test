#ifndef ROBSTRIDE_MOTOR_H
#define ROBSTRIDE_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define ROBSTRIDE_CAN_EXT_ID_MASK             (0x1FFFFFFFU)
#define ROBSTRIDE_EL05_POSITION_MIN_RAD       (-12.57f)
#define ROBSTRIDE_EL05_POSITION_MAX_RAD       (12.57f)
#define ROBSTRIDE_EL05_VELOCITY_MIN_RAD_S     (-50.0f)
#define ROBSTRIDE_EL05_VELOCITY_MAX_RAD_S     (50.0f)
#define ROBSTRIDE_EL05_TORQUE_MIN_NM          (-6.0f)
#define ROBSTRIDE_EL05_TORQUE_MAX_NM          (6.0f)
#define ROBSTRIDE_EL05_KP_MIN                 (0.0f)
#define ROBSTRIDE_EL05_KP_MAX                 (500.0f)
#define ROBSTRIDE_EL05_KD_MIN                 (0.0f)
#define ROBSTRIDE_EL05_KD_MAX                 (5.0f)
#define ROBSTRIDE_DEVICE_ID_BROADCAST_TARGET_ID   (0x7FU)
#define ROBSTRIDE_DEVICE_ID_RESPONSE_TARGET_ID    (0xFEU)
#define ROBSTRIDE_PARAM_CAN_ID                    (0x200AU)
#define ROBSTRIDE_PARAM_CAN_MASTER                (0x200BU)
#define ROBSTRIDE_PARAM_RUN_MODE                  (0x7005U)
#define ROBSTRIDE_PARAM_LOC_KP                    (0x701EU)

typedef enum {
    ROBSTRIDE_COMM_GET_DEVICE_ID = 0,
    ROBSTRIDE_COMM_MOTION_CONTROL = 1,
    ROBSTRIDE_COMM_FEEDBACK = 2,
    ROBSTRIDE_COMM_ENABLE = 3,
    ROBSTRIDE_COMM_DISABLE = 4,
    ROBSTRIDE_COMM_SET_ZERO = 6,
    ROBSTRIDE_COMM_READ_SINGLE_PARAM = 17,
    ROBSTRIDE_COMM_ACTIVE_REPORT = 24,
} RobStride_CommType;

typedef enum {
    ROBSTRIDE_MOTOR_TYPE_EL05 = 1,
} RobStride_Motor_Type;

typedef enum {
    ROBSTRIDE_READ_PARAM_TARGET_CAN_ID = 0,
    ROBSTRIDE_READ_PARAM_TARGET_RUN_MODE = 1,
    ROBSTRIDE_READ_PARAM_TARGET_LOC_KP = 2,
} RobStride_ReadParam_Target;

typedef struct {
    float position_min_rad;
    float position_max_rad;
    float velocity_min_rad_s;
    float velocity_max_rad_s;
    float torque_min_Nm;
    float torque_max_Nm;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
} RobStride_Motor_Params;

typedef struct {
    RobStride_Motor_Type motor_type;
    uint8_t motor_id;
    uint8_t host_can_id;
    RobStride_Motor_Params params;
} RobStride_Motor_Config;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float torque_Nm;
    float kp;
    float kd;
} RobStride_Motor_Command;

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
} RobStride_Motor_Frame;

typedef struct {
    RobStride_Motor_Config config;
    uint8_t motor_id;
    uint8_t mode_state;
    uint8_t fault_code;
    float position_rad;
    float velocity_rad_s;
    float torque_Nm;
    float temperature_c;
    bool is_valid;
} RobStride_Motor_State;

uint16_t RobStride_Motor_FloatToUint16(float value, float min_value, float max_value);
float RobStride_Motor_Uint16ToFloat(uint16_t value, float min_value, float max_value);
uint32_t RobStride_Motor_BuildCanId(RobStride_CommType comm_type, uint16_t data_area2, uint8_t target_id);
uint8_t RobStride_Motor_GetCommType(uint32_t can_id);
uint16_t RobStride_Motor_GetDataArea2(uint32_t can_id);
uint8_t RobStride_Motor_GetTargetId(uint32_t can_id);
RobStride_Motor_Config RobStride_Motor_MakeConfig(RobStride_Motor_Type motor_type,
                                                  uint8_t motor_id,
                                                  uint8_t host_can_id);
void RobStride_Motor_Init(RobStride_Motor_State *motor, const RobStride_Motor_Config *config);
void RobStride_Motor_MakeMotionControlFrame(const RobStride_Motor_Config *config,
                                            const RobStride_Motor_Command *command,
                                            RobStride_Motor_Frame *frame);
void RobStride_Motor_MakeEnableFrame(const RobStride_Motor_Config *config, RobStride_Motor_Frame *frame);
void RobStride_Motor_MakeDisableFrame(const RobStride_Motor_Config *config,
                                      bool clear_fault,
                                      RobStride_Motor_Frame *frame);
void RobStride_Motor_MakeSetZeroFrame(const RobStride_Motor_Config *config, RobStride_Motor_Frame *frame);
void RobStride_Motor_MakeActiveReportFrame(const RobStride_Motor_Config *config,
                                           bool enable_active_report,
                                           RobStride_Motor_Frame *frame);
void RobStride_Motor_MakeGetDeviceIdFrame(const RobStride_Motor_Config *config, RobStride_Motor_Frame *frame);
void RobStride_Motor_MakeReadSingleParamFrame(const RobStride_Motor_Config *config,
                                              uint16_t param_index,
                                              RobStride_Motor_Frame *frame);
bool RobStride_Motor_ParseDeviceIdResponse(uint32_t can_id,
                                           const uint8_t data[8],
                                           uint8_t *motor_id,
                                           uint8_t uid[8]);
bool RobStride_Motor_ParseReadSingleUint8Response(uint32_t can_id,
                                                  const uint8_t data[8],
                                                  uint16_t expected_param_index,
                                                  uint8_t expected_host_can_id,
                                                  uint8_t *value);
bool RobStride_Motor_ParseReadSingleFloat32Response(uint32_t can_id,
                                                    const uint8_t data[8],
                                                    uint16_t expected_param_index,
                                                    uint8_t expected_host_can_id,
                                                    float *value);
bool RobStride_Motor_ParseReadSingleParamFailure(uint32_t can_id,
                                                 const uint8_t data[8],
                                                 uint16_t expected_param_index,
                                                 uint8_t expected_host_can_id,
                                                 uint8_t *failure_status);
uint16_t RobStride_Motor_SelectReadableParamIndex(RobStride_ReadParam_Target target);
bool RobStride_Motor_UpdateFeedback(RobStride_Motor_State *motor,
                                    uint32_t can_id,
                                    const uint8_t data[8]);

#endif
