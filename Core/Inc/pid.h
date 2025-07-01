#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float last_error;
    float output;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);
float PID_Calc(PID_TypeDef *pid, float measured);

#ifdef __cplusplus
}
#endif

#endif // __PID_H

