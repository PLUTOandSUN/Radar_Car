#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEFT 0
#define RIGHT 1

// 编码器相关参数
#define ENCODER_LINES 1000      // 编码器线数
#define GEAR_RATIO 30           // 减速比
#define SAMPLE_TIME_MS 10       // 采样周期(ms)

// 外部定时器句柄声明
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// 函数声明
void Encoder_Init(void);
void Update_Motor_Speed(void);
float GetMotorSpeed(uint8_t motor);
int32_t GetEncoderCount(uint8_t motor);

#ifdef __cplusplus
}
#endif

#endif // __ENCODER_H
