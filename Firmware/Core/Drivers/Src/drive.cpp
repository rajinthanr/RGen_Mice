#include <cmath>
#include "main.h"
#include "stdbool.h"
#include "drive.h"

extern TIM_HandleTypeDef htim3;

// Function to drive with linear and angular speed (m/s, rad/s)
void drive(float speed, float angular_speed) {
    // Example: Convert speed and angular speed to left/right wheel speeds
    // Assume wheel_base is the distance between wheels in meters
    const float wheel_base = 0.2f; // adjust as needed

    float left_speed = speed - (angular_speed * wheel_base / 2.0f);
    float right_speed = speed + (angular_speed * wheel_base / 2.0f);

    // Clamp speeds to motor limits if necessary
    // send to motor controller
    drive_dif(left_speed, right_speed);
}

// Function to enable or disable the drive system
void drive_enable() {
        // Set PB2 high to enable motors
        HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void drive_disable() {
    // Set PB2 low to disable motors
    HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET);
}

// Function to set differential drive speeds (-1 to 1)
void drive_dif(float left_speed, float right_speed) {
    // Clamp values to -1.0 to 1.0
    left_speed = fmaxf(fminf(left_speed, 1.0f), -1.0f);
    right_speed = fmaxf(fminf(right_speed, 1.0f), -1.0f);

    // Convert to PWM (0 to 4095)
    uint16_t left_pwm = (uint16_t)(fabsf(left_speed) * 4095.0f) % 4096;
    uint16_t right_pwm = (uint16_t)(fabsf(right_speed) * 4095.0f) % 4096;

    // Set PWM for left motor (TIM3 CH1 & CH2)
    if (left_speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
    }

    // Set PWM for right motor (TIM3 CH3 & CH4)
    if (right_speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
    }
}
