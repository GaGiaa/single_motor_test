#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robstride_motor.h"
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

    expect_true("motor type EL05", config.motor_type == ROBSTRIDE_MOTOR_TYPE_EL05);
    expect_true("motor id", config.motor_id == 4U);
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

static void test_default_config(void)
{
    expect_u32("default protocol", MOTOR_DEBUG_PROTOCOL, MOTOR_DEBUG_PROTOCOL_DJI);
    expect_u32("default can channel", MOTOR_DEBUG_CAN_CHANNEL, 2U);
    expect_u32("default robstride motor id", ROBSTRIDE_DEBUG_MOTOR_ID, 1U);
    expect_u32("default host id", ROBSTRIDE_DEBUG_HOST_CAN_ID, 0xFDU);
    expect_u32("default robstride type", ROBSTRIDE_DEBUG_MOTOR_TYPE, ROBSTRIDE_MOTOR_TYPE_EL05);
}

int main(void)
{
    test_float_uint16_mapping();
    test_el05_config_ranges();
    test_motion_control_frame();
    test_control_frames();
    test_feedback_parse();
    test_default_config();

    if (g_failures != 0) {
        printf("%d tests failed\n", g_failures);
        return 1;
    }

    printf("all tests passed\n");
    return 0;
}
