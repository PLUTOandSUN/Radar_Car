#include "pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0;
    pid->integral = 0;
    pid->last_error = 0;
    pid->output = 0;
}

float PID_Calc(PID_TypeDef *pid, float measured) {
    float error = pid->setpoint - measured;
    pid->integral += error;
    float derivative = error - pid->last_error;
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->last_error = error;
    return pid->output;
}

