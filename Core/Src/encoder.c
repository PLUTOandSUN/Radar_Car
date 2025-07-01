#include "encoder.h"

// 编码器相关变量
static int32_t encoder_count_left_prev = 32768;
static int32_t encoder_count_right_prev = 32768;
static float motor_speed_left = 0;
static float motor_speed_right = 0;

// 编码器初始化函数
void Encoder_Init(void) {
    // 启动编码器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // 左电机编码器 TIM3
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 右电机编码器 TIM4

    // 初始化编码器计数值到中值，避免溢出
    __HAL_TIM_SET_COUNTER(&htim3, 32768);
    __HAL_TIM_SET_COUNTER(&htim4, 32768);

    encoder_count_left_prev = 32768;
    encoder_count_right_prev = 32768;
    motor_speed_left = 0;
    motor_speed_right = 0;
}

// 获取编码器计数值
int32_t GetEncoderCount(uint8_t motor) {
    if (motor == LEFT) {
        return __HAL_TIM_GET_COUNTER(&htim3);
    } else {
        return __HAL_TIM_GET_COUNTER(&htim4);
    }
}

// 更新电机速度函数
void Update_Motor_Speed(void) {
    int32_t encoder_count_left = __HAL_TIM_GET_COUNTER(&htim3);
    int32_t encoder_count_right = __HAL_TIM_GET_COUNTER(&htim4);

    // 计算编码器增量
    int32_t delta_left = encoder_count_left - encoder_count_left_prev;
    int32_t delta_right = encoder_count_right - encoder_count_right_prev;

    // 处理计数器溢出情况
    if (delta_left > 32767) delta_left -= 65536;
    if (delta_left < -32767) delta_left += 65536;
    if (delta_right > 32767) delta_right -= 65536;
    if (delta_right < -32767) delta_right += 65536;

    // 计算速度 (脉冲/采样周期 -> RPM)
    // 速度 = (脉冲增量 * 60 * 1000) / (编码器线数 * 减速比 * 采样周期ms)
    motor_speed_left = (float)delta_left * 60000.0f / (ENCODER_LINES * GEAR_RATIO * SAMPLE_TIME_MS);
    motor_speed_right = (float)delta_right * 60000.0f / (ENCODER_LINES * GEAR_RATIO * SAMPLE_TIME_MS);

    // 更新上次计数值
    encoder_count_left_prev = encoder_count_left;
    encoder_count_right_prev = encoder_count_right;
}

// 获取电机速度函数
float GetMotorSpeed(uint8_t motor) {
    if (motor == LEFT) {
        return motor_speed_left;
    } else {
        return motor_speed_right;
    }
}
