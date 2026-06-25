#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float I_Outlimit;
    bool isIOutlimit;
    float output_limit;
    float deadband;
} PID_Param_Config;

typedef struct {
    PID_Param_Config params;
    float dt_s;
    float error;
    float last_error;
    float prev_error;
    float p_out;
    float i_out;
    float d_out;
    float output;
} PID_Incremental;

typedef struct {
    PID_Param_Config params;
    float dt_s;
    float integral;
    float last_error;
    float p_out;
    float i_out;
    float d_out;
    float output;
    bool has_last_error;
} PID_Position;

void PID_Incremental_Init(PID_Incremental *pid, const PID_Param_Config *params, float dt_s);
void PID_Incremental_Reset(PID_Incremental *pid);
float PID_Incremental_Calc(PID_Incremental *pid, float target, float feedback);

void PID_Position_Init(PID_Position *pid, const PID_Param_Config *params, float dt_s);
void PID_Position_Reset(PID_Position *pid);
float PID_Position_Calc(PID_Position *pid, float target, float feedback);

#endif
