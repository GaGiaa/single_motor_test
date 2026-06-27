#include <math.h>
#include <stdio.h>

#include "pid.h"

static int g_failures = 0;

static void expect_near(const char *name, float actual, float expected, float tolerance)
{
    if (fabsf(actual - expected) > tolerance) {
        printf("FAIL %s: actual=%f expected=%f tolerance=%f\n", name, actual, expected, tolerance);
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

static PID_Position_Param_Config make_advanced_params(void)
{
    PID_Position_Param_Config params = {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .I_Outlimit = 0.0f,
        .isIOutlimit = false,
        .output_limit = 0.0f,
        .deadband = 0.0f,
        .setpoint_weight_b = 1.0f,
        .setpoint_weight_c = 0.0f,
        .derivative_filter_N = 10.0f,
        .anti_windup_gain = 1.0f,
    };

    return params;
}

static void test_advanced_integral_limit(void)
{
    PID_Position pid;
    PID_Position_Param_Config params = make_advanced_params();

    params.ki = 1.0f;
    params.I_Outlimit = 0.5f;
    params.isIOutlimit = true;

    PID_Position_Init(&pid, &params, 1.0f);
    expect_near("advanced integral limit output", PID_Position_Calc(&pid, 2.0f, 0.0f), 0.5f, 0.0001f);
    expect_near("advanced integral state limited", pid.integral, 0.5f, 0.0001f);
}

static void test_advanced_anti_windup_reduces_integral(void)
{
    PID_Position pid;
    PID_Position_Param_Config params = make_advanced_params();

    params.ki = 1.0f;
    params.output_limit = 1.0f;
    params.anti_windup_gain = 1.0f;

    PID_Position_Init(&pid, &params, 1.0f);
    expect_near("advanced output saturates", PID_Position_Calc(&pid, 10.0f, 0.0f), 1.0f, 0.0001f);
    expect_true("advanced anti windup back-calculates integral", pid.integral < 10.0f);
}

static void test_advanced_derivative_avoids_setpoint_kick(void)
{
    PID_Position pid;
    PID_Position_Param_Config params = make_advanced_params();

    params.kd = 1.0f;
    params.setpoint_weight_c = 0.0f;
    params.derivative_filter_N = 1000.0f;

    PID_Position_Init(&pid, &params, 0.001f);
    expect_near("advanced first derivative output", PID_Position_Calc(&pid, 0.0f, 0.0f), 0.0f, 0.0001f);
    expect_near("advanced setpoint step has no d kick", PID_Position_Calc(&pid, 100.0f, 0.0f), 0.0f, 0.0001f);
}

static void test_advanced_derivative_filters_feedback_step(void)
{
    PID_Position pid;
    PID_Position_Param_Config params = make_advanced_params();

    params.kd = 1.0f;
    params.setpoint_weight_c = 0.0f;
    params.derivative_filter_N = 10.0f;

    PID_Position_Init(&pid, &params, 0.001f);
    (void)PID_Position_Calc(&pid, 0.0f, 0.0f);
    (void)PID_Position_Calc(&pid, 0.0f, 1.0f);
    expect_true("advanced derivative filter reduces raw spike", pid.d_out > -1000.0f && pid.d_out < 0.0f);
}

static void test_advanced_reset_clears_added_state(void)
{
    PID_Position pid;
    PID_Position_Param_Config params = make_advanced_params();

    params.ki = 1.0f;
    params.kd = 1.0f;

    PID_Position_Init(&pid, &params, 0.01f);
    (void)PID_Position_Calc(&pid, 2.0f, 1.0f);
    PID_Position_Reset(&pid);

    expect_near("advanced reset integral", pid.integral, 0.0f, 0.0001f);
    expect_near("advanced reset derivative", pid.filtered_derivative, 0.0f, 0.0001f);
    expect_near("advanced reset output", pid.output, 0.0f, 0.0001f);
    expect_true("advanced reset sample flag", !pid.has_last_sample);
}

int main(void)
{
    test_advanced_integral_limit();
    test_advanced_anti_windup_reduces_integral();
    test_advanced_derivative_avoids_setpoint_kick();
    test_advanced_derivative_filters_feedback_step();
    test_advanced_reset_clears_added_state();

    if (g_failures != 0) {
        printf("%d test(s) failed\n", g_failures);
        return 1;
    }

    printf("all tests passed\n");
    return 0;
}
