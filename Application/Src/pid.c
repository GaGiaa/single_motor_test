#include "pid.h"
#include "app_math.h"

#include <math.h>
#include <stddef.h>

static float pid_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static bool pid_has_positive_limit(float limit)
{
    return isfinite(limit) && limit > 0.0f;
}

void PID_Incremental_Init(PID_Incremental *pid, const PID_Param_Config *params, float dt_s)
{
    if (pid == NULL || params == NULL) {
        return;
    }

    pid->params = *params;
    pid->dt_s = dt_s > 0.0f ? dt_s : 0.001f;
    PID_Incremental_Reset(pid);
}

void PID_Incremental_Reset(PID_Incremental *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
}

float PID_Incremental_Calc(PID_Incremental *pid, float target, float feedback)
{
    if (pid == NULL) {
        return 0.0f;
    }

    float error = target - feedback;
    if (pid_absf(error) <= pid->params.deadband) {
        error = 0.0f;
    }

    pid->error = error;
    pid->p_out = pid->params.kp * (error - pid->last_error);
    pid->i_out = pid->params.ki * error * pid->dt_s;
    pid->d_out = pid->params.kd * (error - 2.0f * pid->last_error + pid->prev_error) / pid->dt_s;

    if (pid->params.isIOutlimit && pid_has_positive_limit(pid->params.I_Outlimit)) {
        pid->i_out = App_Math_ClampFloat(pid->i_out, -pid->params.I_Outlimit, pid->params.I_Outlimit);
    }

    pid->output += pid->p_out + pid->i_out + pid->d_out;
    if (pid_has_positive_limit(pid->params.output_limit)) {
        pid->output = App_Math_ClampFloat(pid->output, -pid->params.output_limit, pid->params.output_limit);
    }

    pid->prev_error = pid->last_error;
    pid->last_error = error;
    return pid->output;
}

void PID_Position_Init(PID_Position *pid, const PID_Param_Config *params, float dt_s)
{
    if (pid == NULL || params == NULL) {
        return;
    }

    pid->params = *params;
    pid->dt_s = dt_s > 0.0f ? dt_s : 0.01f;
    PID_Position_Reset(pid);
}

void PID_Position_Reset(PID_Position *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
    pid->has_last_error = false;
}

float PID_Position_Calc(PID_Position *pid, float target, float feedback)
{
    if (pid == NULL) {
        return 0.0f;
    }

    float error = target - feedback;
    if (pid_absf(error) <= pid->params.deadband) {
        error = 0.0f;
    }

    pid->integral += error * pid->dt_s;
    if (pid->params.isIOutlimit && pid_has_positive_limit(pid->params.I_Outlimit)) {
        pid->integral = App_Math_ClampFloat(pid->integral, -pid->params.I_Outlimit, pid->params.I_Outlimit);
    }

    const float derivative = pid->has_last_error ? ((error - pid->last_error) / pid->dt_s) : 0.0f;
    pid->p_out = pid->params.kp * error;
    pid->i_out = pid->params.ki * pid->integral;
    pid->d_out = pid->params.kd * derivative;
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    if (pid_has_positive_limit(pid->params.output_limit)) {
        pid->output = App_Math_ClampFloat(pid->output, -pid->params.output_limit, pid->params.output_limit);
    }

    pid->last_error = error;
    pid->has_last_error = true;
    return pid->output;
}
