#include "math.h"
#include "drive.h"
#include "main.h"
#include "stdbool.h"
#include "drive.h"
#include "encoder.h"
#include "icm.h"

extern TIM_HandleTypeDef htim3;

// Example global variable
MouseMotionState mouse = {
    .linear_speed = 0.0f,
    .angular_speed = 0.0f,
    .target_linear_speed = 0.0f,
    .target_angular_speed = 0.0f,
    .max_linear_speed = 100.0f,  // Example: 800 mm/s
    .max_angular_speed = 0.0f,   // Example: 360 deg/s
    .max_linear_accel = 200.0f, // Example: 2000 mm/s^2
    .max_angular_accel = 100.0f // Example: 1800 deg/s^2
};

// PID controller structure
typedef struct
{
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} PIDController;

// PID controllers for left and right wheels
static PIDController pid_left = {0.002f, 0.002f, 0.0f, 0.0f, 0.0f};
static PIDController pid_right = {0.002f, 0.002f, 0.0f, 0.0f, 0.0f};

// Target speeds (m/s)
static float target_linear = 0.0f;
static float target_angular = 0.0f;

// Call this periodically (e.g., every 10ms)
#include "delay.h"

void drive_closed_loop_update()
{
    static uint32_t last_time = 0;
    uint32_t now = micros();
    float dt = (last_time == 0) ? 0.01f : (now - last_time) / 1000000.0f; // default 10ms on first call
    last_time = now;

    // Get current wheel speeds from encoders (mm/s)
    float left_measured = getLeftSpeed();   // returns mm/s
    float right_measured = getRightSpeed(); // returns mm/s

    // Get current heading from gyro (deg/s)
    float gyro_angular_deg = get_gyroZ();

    // Convert angular speed from deg/s to mm/s for wheel targets
    const float wheel_base = 54.0f;                              // 54mm
    float target_angular_rad = target_angular * (M_PI / 180.0f); // convert deg/s to rad/s if needed

    float left_target = target_linear - (target_angular_rad * wheel_base / 2.0f);
    float right_target = target_linear + (target_angular_rad * wheel_base / 2.0f);
    // --- Low-pass filter for encoder readings ---
    static float left_measured_filt = 0.0f;
    static float right_measured_filt = 0.0f;
    const float alpha = 0.3f; // Low-pass filter coefficient (0 < alpha < 1)

    left_measured_filt = alpha * left_measured + (1.0f - alpha) * left_measured_filt;
    right_measured_filt = alpha * right_measured + (1.0f - alpha) * right_measured_filt;

    // PID for left wheel with anti-windup
    float error_left = left_target - left_measured_filt;
    pid_left.integral += error_left * dt;

    // Anti-windup: clamp integral term
    const float integral_limit = 200.0f; // Adjust as needed
    if (pid_left.integral > integral_limit)
        pid_left.integral = integral_limit;
    if (pid_left.integral < -integral_limit)
        pid_left.integral = -integral_limit;

    float derivative_left = (error_left - pid_left.prev_error) / dt;
    float output_left = pid_left.kp * error_left + pid_left.ki * pid_left.integral + pid_left.kd * derivative_left;
    pid_left.prev_error = error_left;

    // PID for right wheel with anti-windup
    float error_right = right_target - right_measured_filt;
    pid_right.integral += error_right * dt;

    // Anti-windup: clamp integral term
    if (pid_right.integral > integral_limit)
        pid_right.integral = integral_limit;
    if (pid_right.integral < -integral_limit)
        pid_right.integral = -integral_limit;

    float derivative_right = (error_right - pid_right.prev_error) / dt;
    float output_right = pid_right.kp * error_right + pid_right.ki * pid_right.integral + pid_right.kd * derivative_right;
    pid_right.prev_error = error_right;

    // Clamp outputs to -1.0 to 1.0
    output_left = fmaxf(fminf(output_left, 1.0f), -1.0f);
    output_right = fmaxf(fminf(output_right, 1.0f), -1.0f);

    // Send to motors
    drive_dif(output_left, output_right);
}

// Set closed-loop targets
void drive_set_closed_loop(float linear, float angular)
{
    target_linear = linear;
    target_angular = angular;
    // If speed varies over 100 mm/s, reset everything to zero
    if (fabsf(linear - mouse.linear_speed) > 100.0f)
    {
        pid_left.integral = 0.0f;
        pid_left.prev_error = 0.0f;
        pid_right.integral = 0.0f;
        pid_right.prev_error = 0.0f;
    }
}

// Function to drive with linear and angular speed (m/s, rad/s)
void drive(float speed, float angular_speed)
{
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
void drive_enable()
{
    // Set PB2 high to enable motors
    HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void drive_disable()
{
    // Set PB2 low to disable motors
    HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET);
}

// Function to set differential drive speeds (-1 to 1)
void drive_dif(float left_speed, float right_speed)
{
    // Clamp values to -1.0 to 1.0
    left_speed = fmaxf(fminf(left_speed, 1.0f), -1.0f);
    right_speed = fmaxf(fminf(right_speed, 1.0f), -1.0f);

    // Convert to PWM (0 to 4095)
    uint16_t left_pwm = (uint16_t)(fabsf(left_speed) * 4095.0f) % 4096;
    uint16_t right_pwm = (uint16_t)(fabsf(right_speed) * 4095.0f) % 4096;

    // Set PWM for left motor (TIM3 CH1 & CH2)
    if (left_speed < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
    }

    // Set PWM for right motor (TIM3 CH3 & CH4)
    if (right_speed < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
    }
}
