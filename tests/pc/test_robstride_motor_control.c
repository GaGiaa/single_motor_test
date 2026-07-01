#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robstride_motor.h"
#include "robstride_motor_task.h"
#include "robstride_motor_debug_config.h"
#include "motor_debug_config.h"

static int g_failures = 0;

static void expect_u16(const char *name, uint16_t actual, uint16_t expected)
{
    if (actual != expected) {
        printf("FAIL %s: actual=%u expected=%u\n", name, actual, expected);
        ++g_failures;
    }
}

static void expect_u32(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual != expected) {
        printf("FAIL %s: actual=0x%08lX expected=0x%08lX\n",
               name,
               (unsigned long)actual,
               (unsigned long)expected);
        ++g_failures;
    }
}

static void expect_near(const char *name, float actual, float expected, float tolerance)
{
    if (fabsf(actual - expected) > tolerance) {
        printf("FAIL %s: actual=%f expected=%f tolerance=%f\n",
               name,
               actual,
               expected,
               tolerance);
        ++g_failures;
    }
}

static void expect_true(const char *name, int condition)
{
    if (!condition) {
        printf("FAIL %s\n", name);
        ++g_failures;
    }
}

static uint16_t be_u16(const uint8_t data[8], uint8_t index)
{
    return (uint16_t)(((uint16_t)data[index] << 8) | data[index + 1U]);
}

static uint16_t le_u16(const uint8_t data[8], uint8_t index)
{
    return (uint16_t)(((uint16_t)data[index + 1U] << 8) | data[index]);
}

static void test_float_uint16_mapping(void)
{
    expect_u16("min maps to 0", RobStride_Motor_FloatToUint16(-10.0f, -10.0f, 10.0f), 0U);
    expect_u16("max maps to 65535", RobStride_Motor_FloatToUint16(10.0f, -10.0f, 10.0f), 65535U);
    expect_true("mid maps near 32768",
                RobStride_Motor_FloatToUint16(0.0f, -10.0f, 10.0f) >= 32767U &&
                RobStride_Motor_FloatToUint16(0.0f, -10.0f, 10.0f) <= 32768U);
    expect_u16("below min clamps", RobStride_Motor_FloatToUint16(-20.0f, -10.0f, 10.0f), 0U);
    expect_u16("above max clamps", RobStride_Motor_FloatToUint16(20.0f, -10.0f, 10.0f), 65535U);
    expect_near("raw 0 maps to min", RobStride_Motor_Uint16ToFloat(0U, -10.0f, 10.0f), -10.0f, 0.001f);
    expect_near("raw 65535 maps to max", RobStride_Motor_Uint16ToFloat(65535U, -10.0f, 10.0f), 10.0f, 0.001f);
}

static void test_el05_config_ranges(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_Config extended_id_config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 0x7FU, 0xFDU);

    expect_true("motor type EL05", config.motor_type == ROBSTRIDE_MOTOR_TYPE_EL05);
    expect_true("motor id", config.motor_id == 4U);
    expect_true("extended motor id 0x7F accepted", extended_id_config.motor_id == 0x7FU);
    expect_true("host id", config.host_can_id == 0xFDU);
    expect_near("position min", config.params.position_min_rad, -12.57f, 0.0001f);
    expect_near("position max", config.params.position_max_rad, 12.57f, 0.0001f);
    expect_near("velocity min", config.params.velocity_min_rad_s, -50.0f, 0.0001f);
    expect_near("velocity max", config.params.velocity_max_rad_s, 50.0f, 0.0001f);
    expect_near("torque min", config.params.torque_min_Nm, -6.0f, 0.0001f);
    expect_near("torque max", config.params.torque_max_Nm, 6.0f, 0.0001f);
    expect_near("kp max", config.params.kp_max, 500.0f, 0.0001f);
    expect_near("kd max", config.params.kd_max, 5.0f, 0.0001f);
}

static void test_motion_control_frame(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_Command command = {
        .position_rad = 0.0f,
        .velocity_rad_s = 25.0f,
        .torque_Nm = 3.0f,
        .kp = 250.0f,
        .kd = 2.5f,
    };
    RobStride_Motor_Frame frame;

    RobStride_Motor_MakeMotionControlFrame(&config, &command, &frame);

    expect_u32("motion comm type", RobStride_Motor_GetCommType(frame.can_id), ROBSTRIDE_COMM_MOTION_CONTROL);
    expect_u32("motion target", RobStride_Motor_GetTargetId(frame.can_id), 4U);
    expect_u32("motion torque in data area2",
               RobStride_Motor_GetDataArea2(frame.can_id),
               RobStride_Motor_FloatToUint16(3.0f, -6.0f, 6.0f));
    expect_u16("motion pos raw", be_u16(frame.data, 0U), RobStride_Motor_FloatToUint16(0.0f, -12.57f, 12.57f));
    expect_u16("motion vel raw", be_u16(frame.data, 2U), RobStride_Motor_FloatToUint16(25.0f, -50.0f, 50.0f));
    expect_u16("motion kp raw", be_u16(frame.data, 4U), RobStride_Motor_FloatToUint16(250.0f, 0.0f, 500.0f));
    expect_u16("motion kd raw", be_u16(frame.data, 6U), RobStride_Motor_FloatToUint16(2.5f, 0.0f, 5.0f));
}

static void test_control_frames(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_Frame frame;

    RobStride_Motor_MakeEnableFrame(&config, &frame);
    expect_u32("enable comm type", RobStride_Motor_GetCommType(frame.can_id), ROBSTRIDE_COMM_ENABLE);
    expect_u32("enable data area2 host", RobStride_Motor_GetDataArea2(frame.can_id), 0xFDU);
    expect_true("enable data zero", memcmp(frame.data, "\0\0\0\0\0\0\0\0", 8U) == 0);

    RobStride_Motor_MakeDisableFrame(&config, true, &frame);
    expect_u32("disable comm type", RobStride_Motor_GetCommType(frame.can_id), ROBSTRIDE_COMM_DISABLE);
    expect_true("disable clear fault byte", frame.data[0] == 1U);

    RobStride_Motor_MakeSetZeroFrame(&config, &frame);
    expect_u32("set zero comm type", RobStride_Motor_GetCommType(frame.can_id), ROBSTRIDE_COMM_SET_ZERO);
    expect_true("set zero byte", frame.data[0] == 1U);
}

static void test_active_report_frame(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_Frame frame;

    RobStride_Motor_MakeActiveReportFrame(&config, true, &frame);

    expect_u32("active report comm type", RobStride_Motor_GetCommType(frame.can_id), ROBSTRIDE_COMM_ACTIVE_REPORT);
    expect_u32("active report host", RobStride_Motor_GetDataArea2(frame.can_id), 0xFDU);
    expect_u32("active report target", RobStride_Motor_GetTargetId(frame.can_id), ROBSTRIDE_DEVICE_ID_BROADCAST_TARGET_ID);
    expect_true("active report data header",
                frame.data[0] == 0x01U &&
                frame.data[1] == 0x02U &&
                frame.data[2] == 0x03U &&
                frame.data[3] == 0x04U &&
                frame.data[4] == 0x05U &&
                frame.data[5] == 0x06U);
    expect_true("active report enable flag", frame.data[6] == 0x01U);
    expect_true("active report tail zero", frame.data[7] == 0x00U);

    RobStride_Motor_MakeActiveReportFrame(&config, false, &frame);
    expect_true("active report disable flag", frame.data[6] == 0x00U);
}

static void test_get_device_id_frame(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_Frame frame;

    RobStride_Motor_MakeGetDeviceIdFrame(&config, &frame);

    expect_u32("get device id comm type", RobStride_Motor_GetCommType(frame.can_id), ROBSTRIDE_COMM_GET_DEVICE_ID);
    expect_u32("get device id host", RobStride_Motor_GetDataArea2(frame.can_id), 0xFDU);
    expect_u32("get device id target", RobStride_Motor_GetTargetId(frame.can_id), ROBSTRIDE_DEVICE_ID_BROADCAST_TARGET_ID);
    expect_true("get device id data zero", memcmp(frame.data, "\0\0\0\0\0\0\0\0", 8U) == 0);
}

static void test_get_device_id_response_parse(void)
{
    const uint8_t uid[8] = {0x10U, 0x32U, 0x54U, 0x76U, 0x98U, 0xBAU, 0xDCU, 0xFEU};
    uint8_t parsed_motor_id = 0U;
    uint8_t parsed_uid[8] = {0};
    uint32_t valid_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_GET_DEVICE_ID, 0x0005U, ROBSTRIDE_DEVICE_ID_RESPONSE_TARGET_ID);
    uint32_t wrong_comm_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_FEEDBACK, 0x0005U, ROBSTRIDE_DEVICE_ID_RESPONSE_TARGET_ID);
    uint32_t wrong_target_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_GET_DEVICE_ID, 0x0005U, 0x01U);

    expect_true("parse valid device id response",
                RobStride_Motor_ParseDeviceIdResponse(valid_can_id, uid, &parsed_motor_id, parsed_uid));
    expect_u32("parsed discovered motor id", parsed_motor_id, 5U);
    expect_true("parsed uid copied", memcmp(parsed_uid, uid, sizeof(parsed_uid)) == 0);
    expect_true("reject wrong comm type",
                !RobStride_Motor_ParseDeviceIdResponse(wrong_comm_can_id, uid, &parsed_motor_id, parsed_uid));
    expect_true("reject wrong target id",
                !RobStride_Motor_ParseDeviceIdResponse(wrong_target_can_id, uid, &parsed_motor_id, parsed_uid));
}

static void test_read_single_param_frame(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_Frame frame;

    RobStride_Motor_MakeReadSingleParamFrame(&config, ROBSTRIDE_PARAM_CAN_ID, &frame);

    expect_u32("read param comm type", RobStride_Motor_GetCommType(frame.can_id), ROBSTRIDE_COMM_READ_SINGLE_PARAM);
    expect_u32("read param host", RobStride_Motor_GetDataArea2(frame.can_id), 0xFDU);
    expect_u32("read param target", RobStride_Motor_GetTargetId(frame.can_id), 4U);
    expect_u16("read param index le", le_u16(frame.data, 0U), ROBSTRIDE_PARAM_CAN_ID);
    expect_true("read param index bytes little endian", frame.data[0] == 0x0AU && frame.data[1] == 0x20U);
    expect_true("read param reserved zero", frame.data[2] == 0U && frame.data[3] == 0U);
    expect_true("read param payload zero", frame.data[4] == 0U &&
                                          frame.data[5] == 0U &&
                                          frame.data[6] == 0U &&
                                          frame.data[7] == 0U);
}

static void test_read_single_param_response_parse(void)
{
    uint8_t value_u8 = 0U;
    uint32_t valid_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x007FU, 0xFDU);
    uint32_t valid_non_default_motor_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x0004U, 0xFDU);
    uint32_t fail_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x017FU, 0xFDU);
    uint8_t valid_data[8] = {0x0AU, 0x20U, 0x00U, 0x00U, 0x7FU, 0x00U, 0x00U, 0x00U};
    uint8_t valid_non_default_motor_data[8] = {0x0AU, 0x20U, 0x00U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U};
    uint8_t wrong_index_data[8] = {0x0BU, 0x20U, 0x00U, 0x00U, 0x7FU, 0x00U, 0x00U, 0x00U};

    expect_true("parse read param can id response",
                RobStride_Motor_ParseReadSingleUint8Response(valid_can_id,
                                                             valid_data,
                                                             ROBSTRIDE_PARAM_CAN_ID,
                                                             0xFDU,
                                                             0x7FU,
                                                             &value_u8));
    expect_u32("parsed can id value", value_u8, 0x7FU);
    expect_true("reject read param failure status",
                !RobStride_Motor_ParseReadSingleUint8Response(fail_can_id,
                                                              valid_data,
                                                              ROBSTRIDE_PARAM_CAN_ID,
                                                              0xFDU,
                                                              0x7FU,
                                                              &value_u8));
    expect_true("reject read param wrong index",
                !RobStride_Motor_ParseReadSingleUint8Response(valid_can_id,
                                                              wrong_index_data,
                                                              ROBSTRIDE_PARAM_CAN_ID,
                                                              0xFDU,
                                                              0x7FU,
                                                              &value_u8));
    expect_true("parse read param non-default motor id response",
                RobStride_Motor_ParseReadSingleUint8Response(valid_non_default_motor_can_id,
                                                             valid_non_default_motor_data,
                                                             ROBSTRIDE_PARAM_CAN_ID,
                                                             0xFDU,
                                                             0x04U,
                                                             &value_u8));
    expect_u32("parsed non-default motor can id value", value_u8, 0x04U);
}

static void test_read_single_param_float_response_parse(void)
{
    float value_f32 = 0.0f;
    uint32_t valid_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x007FU, 0xFDU);
    uint32_t fail_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x017FU, 0xFDU);
    uint8_t valid_data[8] = {0x1EU, 0x70U, 0x00U, 0x00U, 0x00U, 0x00U, 0x20U, 0x41U};
    uint8_t wrong_index_data[8] = {0x05U, 0x70U, 0x00U, 0x00U, 0x00U, 0x00U, 0x20U, 0x41U};

    expect_true("parse read param float response",
                RobStride_Motor_ParseReadSingleFloat32Response(valid_can_id,
                                                               valid_data,
                                                               ROBSTRIDE_PARAM_LOC_KP,
                                                               0xFDU,
                                                               0x7FU,
                                                               &value_f32));
    expect_near("parsed loc kp value", value_f32, 10.0f, 0.0001f);
    expect_true("reject float response failure status",
                !RobStride_Motor_ParseReadSingleFloat32Response(fail_can_id,
                                                                valid_data,
                                                                ROBSTRIDE_PARAM_LOC_KP,
                                                                0xFDU,
                                                                0x7FU,
                                                                &value_f32));
    expect_true("reject float response wrong index",
                !RobStride_Motor_ParseReadSingleFloat32Response(valid_can_id,
                                                                wrong_index_data,
                                                                ROBSTRIDE_PARAM_LOC_KP,
                                                                0xFDU,
                                                                0x7FU,
                                                                &value_f32));
}

static void test_read_single_param_failure_parse(void)
{
    uint8_t failure_status = 0U;
    uint32_t fail_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x017FU, 0xFDU);
    uint32_t fail_non_default_motor_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x0104U, 0xFDU);
    uint32_t success_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x007FU, 0xFDU);
    uint32_t wrong_target_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_READ_SINGLE_PARAM, 0x017FU, 0xFEU);
    uint8_t valid_data[8] = {0x0AU, 0x20U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    uint8_t wrong_index_data[8] = {0x05U, 0x70U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

    expect_true("parse read param failure response",
                RobStride_Motor_ParseReadSingleParamFailure(fail_can_id,
                                                            valid_data,
                                                            ROBSTRIDE_PARAM_CAN_ID,
                                                            0xFDU,
                                                            0x7FU,
                                                            &failure_status));
    expect_u32("parsed read param failure status", failure_status, 1U);
    expect_true("reject success response as failure",
                !RobStride_Motor_ParseReadSingleParamFailure(success_can_id,
                                                             valid_data,
                                                             ROBSTRIDE_PARAM_CAN_ID,
                                                             0xFDU,
                                                             0x7FU,
                                                             &failure_status));
    expect_true("reject wrong response target host",
                !RobStride_Motor_ParseReadSingleParamFailure(wrong_target_can_id,
                                                             valid_data,
                                                             ROBSTRIDE_PARAM_CAN_ID,
                                                             0xFDU,
                                                             0x7FU,
                                                             &failure_status));
    expect_true("reject wrong failure response index",
                !RobStride_Motor_ParseReadSingleParamFailure(fail_can_id,
                                                             wrong_index_data,
                                                             ROBSTRIDE_PARAM_CAN_ID,
                                                             0xFDU,
                                                             0x7FU,
                                                             &failure_status));
    expect_true("parse read param non-default motor failure response",
                RobStride_Motor_ParseReadSingleParamFailure(fail_non_default_motor_can_id,
                                                            valid_data,
                                                            ROBSTRIDE_PARAM_CAN_ID,
                                                            0xFDU,
                                                            0x04U,
                                                            &failure_status));
    expect_u32("parsed non-default motor read param failure status", failure_status, 1U);
}

static void test_select_readable_param_index(void)
{
    expect_u16("read target can id", RobStride_Motor_SelectReadableParamIndex(ROBSTRIDE_READ_PARAM_TARGET_CAN_ID), ROBSTRIDE_PARAM_CAN_ID);
    expect_u16("read target run mode", RobStride_Motor_SelectReadableParamIndex(ROBSTRIDE_READ_PARAM_TARGET_RUN_MODE), ROBSTRIDE_PARAM_RUN_MODE);
    expect_u16("read target loc kp", RobStride_Motor_SelectReadableParamIndex(ROBSTRIDE_READ_PARAM_TARGET_LOC_KP), ROBSTRIDE_PARAM_LOC_KP);
    expect_u16("fallback read target", RobStride_Motor_SelectReadableParamIndex((RobStride_ReadParam_Target)99), ROBSTRIDE_PARAM_CAN_ID);
}

static void test_feedback_parse(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_State state;
    uint8_t data[8] = {0};
    uint32_t can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_FEEDBACK,
                                                 (uint16_t)((2U << 14) | (0x15U << 8) | 4U),
                                                 0U);

    data[0] = 0x80U;
    data[1] = 0x00U;
    data[2] = 0xBFU;
    data[3] = 0xFFU;
    data[4] = 0x80U;
    data[5] = 0x00U;
    data[6] = 0x01U;
    data[7] = 0x2CU;

    RobStride_Motor_Init(&state, &config);
    expect_true("feedback accepted", RobStride_Motor_UpdateFeedback(&state, can_id, data));
    expect_true("feedback valid", state.is_valid);
    expect_true("feedback motor id", state.motor_id == 4U);
    expect_true("feedback fault", state.fault_code == 0x15U);
    expect_true("feedback mode state", state.mode_state == 2U);
    expect_near("feedback position", state.position_rad, 0.0f, 0.001f);
    expect_near("feedback velocity", state.velocity_rad_s, 25.0f, 0.02f);
    expect_near("feedback torque", state.torque_Nm, 0.0f, 0.001f);
    expect_near("feedback temp", state.temperature_c, 30.0f, 0.001f);
}

static void test_active_report_feedback_parse(void)
{
    RobStride_Motor_Config config = RobStride_Motor_MakeConfig(ROBSTRIDE_MOTOR_TYPE_EL05, 4U, 0xFDU);
    RobStride_Motor_State state;
    RobStride_Motor_State wrong_host_state;
    uint8_t data[8] = {0};
    uint32_t can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_ACTIVE_REPORT,
                                                 (uint16_t)((2U << 14) | (0x15U << 8) | 4U),
                                                 0xFDU);
    uint32_t wrong_host_can_id = RobStride_Motor_BuildCanId(ROBSTRIDE_COMM_ACTIVE_REPORT,
                                                            (uint16_t)((2U << 14) | (0x15U << 8) | 4U),
                                                            0xFEU);

    data[0] = 0x80U;
    data[1] = 0x00U;
    data[2] = 0xBFU;
    data[3] = 0xFFU;
    data[4] = 0x80U;
    data[5] = 0x00U;
    data[6] = 0x01U;
    data[7] = 0x2CU;

    RobStride_Motor_Init(&state, &config);
    RobStride_Motor_Init(&wrong_host_state, &config);
    expect_true("active report accepted", RobStride_Motor_UpdateFeedback(&state, can_id, data));
    expect_true("active report valid", state.is_valid);
    expect_true("active report motor id", state.motor_id == 4U);
    expect_true("active report fault", state.fault_code == 0x15U);
    expect_true("active report mode state", state.mode_state == 2U);
    expect_near("active report position", state.position_rad, 0.0f, 0.001f);
    expect_near("active report velocity", state.velocity_rad_s, 25.0f, 0.02f);
    expect_near("active report torque", state.torque_Nm, 0.0f, 0.001f);
    expect_near("active report temp", state.temperature_c, 30.0f, 0.001f);
    expect_true("active report rejects wrong host",
                !RobStride_Motor_UpdateFeedback(&wrong_host_state, wrong_host_can_id, data));
}

static void test_default_config(void)
{
    expect_u32("default protocol", MOTOR_DEBUG_PROTOCOL, MOTOR_DEBUG_PROTOCOL_ROBSTRIDE);
    expect_u32("default can channel", MOTOR_DEBUG_CAN_CHANNEL, 2U);
    expect_u32("default robstride motor id", ROBSTRIDE_DEBUG_MOTOR_ID, 0x7FU);
    expect_u32("default host id", ROBSTRIDE_DEBUG_HOST_CAN_ID, 0xFDU);
    expect_u32("default robstride type", ROBSTRIDE_DEBUG_MOTOR_TYPE, ROBSTRIDE_MOTOR_TYPE_EL05);
}

static void test_type17_read_debug_group_layout(void)
{
    RobStride_Motor_Debug debug;

    memset(&debug, 0, sizeof(debug));
    debug.type17_read.request = true;
    debug.type17_read.param_target = ROBSTRIDE_READ_PARAM_TARGET_RUN_MODE;
    debug.type17_read.param_index = ROBSTRIDE_PARAM_RUN_MODE;
    debug.type17_read.request_count = 3U;
    debug.type17_read.success_count = 2U;
    debug.type17_read.fail_count = 1U;
    debug.type17_read.unhandled_count = 4U;
    debug.type17_read.last_u8_value = 0x7FU;
    debug.type17_read.last_u8_value_valid = true;
    debug.type17_read.last_fail_status = 1U;
    debug.type17_read.last_tx_can_id = 0x11111111U;
    debug.type17_read.last_success_rx_can_id = 0x22222222U;
    debug.type17_read.last_fail_rx_can_id = 0x33333333U;

    expect_true("type17 request writable", debug.type17_read.request);
    expect_u32("type17 param target writable", debug.type17_read.param_target, ROBSTRIDE_READ_PARAM_TARGET_RUN_MODE);
    expect_u16("type17 param index writable", debug.type17_read.param_index, ROBSTRIDE_PARAM_RUN_MODE);
    expect_u32("type17 request count writable", debug.type17_read.request_count, 3U);
    expect_u32("type17 success count writable", debug.type17_read.success_count, 2U);
    expect_u32("type17 fail count writable", debug.type17_read.fail_count, 1U);
    expect_u32("type17 unhandled count writable", debug.type17_read.unhandled_count, 4U);
    expect_u32("type17 last u8 value writable", debug.type17_read.last_u8_value, 0x7FU);
    expect_true("type17 last u8 value valid writable", debug.type17_read.last_u8_value_valid);
    expect_u32("type17 fail status writable", debug.type17_read.last_fail_status, 1U);
    expect_u32("type17 last tx can id writable", debug.type17_read.last_tx_can_id, 0x11111111U);
    expect_u32("type17 last success rx can id writable", debug.type17_read.last_success_rx_can_id, 0x22222222U);
    expect_u32("type17 last fail rx can id writable", debug.type17_read.last_fail_rx_can_id, 0x33333333U);
}

int main(void)
{
    test_float_uint16_mapping();
    test_el05_config_ranges();
    test_motion_control_frame();
    test_control_frames();
    test_active_report_frame();
    test_get_device_id_frame();
    test_get_device_id_response_parse();
    test_read_single_param_frame();
    test_read_single_param_response_parse();
    test_read_single_param_float_response_parse();
    test_read_single_param_failure_parse();
    test_select_readable_param_index();
    test_feedback_parse();
    test_active_report_feedback_parse();
    test_default_config();
    test_type17_read_debug_group_layout();

    if (g_failures != 0) {
        printf("%d tests failed\n", g_failures);
        return 1;
    }

    printf("all tests passed\n");
    return 0;
}
