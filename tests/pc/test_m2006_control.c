#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pid.h"
#include "dji_motor.h"

static int g_failures = 0;

static void expect_near(const char *name, float actual, float expected, float tolerance)
{
    if (fabsf(actual - expected) > tolerance) {
        printf("FAIL %s: actual=%f expected=%f tolerance=%f\n", name, actual, expected, tolerance);
        ++g_failures;
    }
}

static void expect_i16(const char *name, int16_t actual, int16_t expected)
{
    if (actual != expected) {
        printf("FAIL %s: actual=%d expected=%d\n", name, actual, expected);
        ++g_failures;
    }
}

static void expect_u8(const char *name, uint8_t actual, uint8_t expected)
{
    if (actual != expected) {
        printf("FAIL %s: actual=%u expected=%u\n", name, (unsigned)actual, (unsigned)expected);
        ++g_failures;
    }
}

static void test_incremental_pid_uses_real_dt(void)
{
    PID_Incremental pid;
    PID_Param_Config params = {
        .kp = 0.004f,
        .ki = 0.35f,
        .kd = 0.0f,
        .I_Outlimit = 1.5f,
        .isIOutlimit = true,
        .output_limit = 2.0f,
        .deadband = 3.0f,
    };

    PID_Incremental_Init(&pid, &params, 0.001f);
    expect_near("speed deadband zeroes small error", PID_Incremental_Calc(&pid, 2.0f, 0.0f), 0.0f, 0.0001f);
    expect_near("speed first step output", PID_Incremental_Calc(&pid, 100.0f, 0.0f), 0.435f, 0.0001f);
    expect_near("speed second step accumulates integral only", PID_Incremental_Calc(&pid, 100.0f, 0.0f), 0.470f, 0.0001f);
}

static void test_position_pid_uses_real_dt(void)
{
    PID_Position pid;
    PID_Param_Config params = {
        .kp = 1.5f,
        .ki = 0.0f,
        .kd = 0.0f,
        .I_Outlimit = 0.0f,
        .isIOutlimit = true,
        .output_limit = 120.0f,
        .deadband = 1.0f,
    };

    PID_Position_Init(&pid, &params, 0.01f);
    expect_near("position deadband zeroes small error", PID_Position_Calc(&pid, 0.5f, 0.0f), 0.0f, 0.0001f);
    expect_near("position p output", PID_Position_Calc(&pid, 20.0f, 0.0f), 30.0f, 0.0001f);
    expect_near("position output limit", PID_Position_Calc(&pid, 200.0f, 0.0f), 120.0f, 0.0001f);
}

static void test_dji_feedback_and_current_conversion(void)
{
    DJI_Motor_State motor;
    DJI_Motor_Config config = DJI_Motor_MakeConfig(DJI_MOTOR_TYPE_M2006, 1U);
    uint8_t feedback[8] = {0x20, 0x00, 0x03, 0xE8, 0x20, 0x00, 55, 0};

    DJI_Motor_Init(&motor, &config);
    DJI_Motor_UpdateFeedback(&motor, feedback);

    expect_near("single turn angle deg", motor.feedback_angle_deg, 10.0f, 0.01f);
    expect_near("output speed rpm", motor.feedback_speed_rpm, 1000.0f / 36.0f, 0.01f);
    expect_near("current raw to A", motor.feedback_current_A, 5.0f, 0.01f);
    expect_u8("temperature", motor.feedback_temperature_c, 55);

    expect_i16("positive current A to raw", DJI_Motor_CurrentAToRaw(&config, 2.0f), 3277);
    expect_i16("negative current A to raw", DJI_Motor_CurrentAToRaw(&config, -2.0f), -3277);
    expect_i16("current clamps high", DJI_Motor_CurrentAToRaw(&config, 100.0f), 16384);
}

static void test_dji_pack_slot_one(void)
{
    uint8_t data[8];
    DJI_Motor_Config config = DJI_Motor_MakeConfig(DJI_MOTOR_TYPE_M2006, 1U);
    memset(data, 0xAA, sizeof(data));
    DJI_Motor_PackCurrentFrame(&config, DJI_Motor_CurrentAToRaw(&config, 1.0f), data);

    expect_u8("slot1 high", data[0], 0x06);
    expect_u8("slot1 low", data[1], 0x66);
    for (int i = 2; i < 8; ++i) {
        expect_u8("unused slots zero", data[i], 0);
    }
}

static void test_m3508_feedback_and_current_conversion(void)
{
    DJI_Motor_State motor;
    DJI_Motor_Config config = DJI_Motor_MakeConfig(DJI_MOTOR_TYPE_M3508, 8U);
    uint8_t feedback[8] = {0x10, 0x00, 0x03, 0xE8, 0x10, 0x00, 60, 0};
    const float m3508_ratio = 3591.0f / 187.0f;

    DJI_Motor_Init(&motor, &config);
    DJI_Motor_UpdateFeedback(&motor, feedback);

    expect_near("m3508 single turn angle deg",
                motor.feedback_angle_deg,
                4096.0f * 360.0f / (8192.0f * m3508_ratio),
                0.01f);
    expect_near("m3508 output speed rpm", motor.feedback_speed_rpm, 1000.0f / m3508_ratio, 0.01f);
    expect_near("c620 current raw to A", motor.feedback_current_A, 5.0f, 0.01f);
    expect_i16("c620 positive current A to raw", DJI_Motor_CurrentAToRaw(&config, 2.0f), 1638);
    expect_i16("c620 current clamps high", DJI_Motor_CurrentAToRaw(&config, 100.0f), 16384);
}

static void test_dji_id_to_can_ids_and_slots(void)
{
    for (uint8_t id = 1U; id <= 8U; ++id) {
        DJI_Motor_Config config = DJI_Motor_MakeConfig(DJI_MOTOR_TYPE_M2006, id);
        uint8_t data[8];
        uint8_t slot = (uint8_t)((id - 1U) % 4U);
        uint8_t byte_index = (uint8_t)(slot * 2U);

        expect_i16("feedback id", (int16_t)config.feedback_id, (int16_t)(0x200U + id));
        expect_i16("control id",
                   (int16_t)config.control_id,
                   (int16_t)(id <= 4U ? 0x200U : 0x1FFU));
        expect_u8("control slot", config.control_slot, slot);

        memset(data, 0xAA, sizeof(data));
        DJI_Motor_PackCurrentFrame(&config, 0x1234, data);
        for (uint8_t i = 0U; i < 8U; ++i) {
            uint8_t expected = 0U;
            if (i == byte_index) {
                expected = 0x12U;
            } else if (i == (uint8_t)(byte_index + 1U)) {
                expected = 0x34U;
            }
            expect_u8("slot packing", data[i], expected);
        }
    }
}

int main(void)
{
    test_incremental_pid_uses_real_dt();
    test_position_pid_uses_real_dt();
    test_dji_feedback_and_current_conversion();
    test_dji_pack_slot_one();
    test_m3508_feedback_and_current_conversion();
    test_dji_id_to_can_ids_and_slots();

    if (g_failures != 0) {
        printf("%d test(s) failed\n", g_failures);
        return 1;
    }

    printf("all tests passed\n");
    return 0;
}

