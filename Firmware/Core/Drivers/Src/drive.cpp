#include "math.h"
#include "drive.h"
#include "main.h"
#include "stdbool.h"
#include "drive.h"
#include "encoder.h"
#include "icm.h"
#include "sensor_Function.h"
#include "mouse.h"
#include "core.h"

extern TIM_HandleTypeDef htim3;

uint8_t debug_mot = 0;
uint8_t is_left_wall = 0;
uint8_t is_right_wall = 0;


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
PIDController pid_left = {0.002f, 0.002f, 0.0f, 0.0f, 0.0f};
PIDController pid_right = {0.002f, 0.002f, 0.0f, 0.0f, 0.0f};

// Target speeds (m/s)
static float target_linear = 0.0f;
static float target_angular = 0.0f;

// Call this periodically (e.g., every 10ms)
#include "delay.h"

float position_controller() {
    static float previous_error = 0.0f;
    static float I = 0.0f;
    const float KP = 0.4f; // Proportional gain
    const float KD = 0.1f; // Derivative gain
    const float KI = 0.3f;  // Integral gain

    mouse.target_dis += (mouse.linear_speed + mouse.speed_adj) * LOOP_INTERVAL;
    mouse.speed_adj = 0; // Reset after use
    float error = mouse.target_dis - get_forward_dis();
    static float filtered_error = 0.0f;
    const float alpha = 0.5f; // Low-pass filter coefficient (0 < alpha < 1)
    filtered_error = alpha * error + (1.0f - alpha) * filtered_error;
    error = filtered_error;
    float diff = error - previous_error;
    previous_error = error;
    I += error * LOOP_INTERVAL;

    I = fmaxf(fminf(I, 3.0f), -3.0f); // Anti-windup using clamp

    float output = KP * error + KD * diff + KI * I;
    return output;
  }

  float angle_controller() {
    static float previous_error = 0.0f;
    static float I = 0.0f;
    const float KP = 0.03f; // Proportional gain
    const float KD = 2.01f; // Derivative gain
    const float KI = 0.05f;  // Integral gain

    float LOOP_INTERVAL = 0.001;
    
    mouse.target_angle += mouse.angular_speed  * LOOP_INTERVAL;
    if(mouse.steering_mode != STEERING_OFF){
        wallFollow(is_left_wall, is_right_wall);
    mouse.target_angle +=  mouse.steering_adjustment * LOOP_INTERVAL;
    }
    if(mouse.is_front_adjust){
       mouse.wall_error = wallFront();
    mouse.target_angle +=  mouse.front_adjustment * LOOP_INTERVAL;
    }

    mouse.steering_adjustment = 0; // Reset after use
    float error = mouse.target_angle - angle;
    static float filtered_error = 0.0f;
    const float alpha = 0.2f; // Low-pass filter coefficient (0 < alpha < 1)
    filtered_error = alpha * error + (1.0f - alpha) * filtered_error;
    error = filtered_error;
    float diff = error - previous_error;
    previous_error = error;
    I += error * LOOP_INTERVAL;

    if(I > 3) I = 3; // Anti-windup
    if(I < -3) I = -3;

    float output = KP * error + KD * diff + KI * I;
    return output;
  }

void drive_closed_loop_update()
{
    float angle_correction = 0;
    float position_correction = 0;

    if(mouse.steering_mode == GYRO_OFF) angle_correction = 0;
    else angle_correction = angle_controller();
    position_correction = position_controller();

    float output_left = position_correction - angle_correction;
    float output_right = position_correction + angle_correction;

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
void drive_init()
{
    // Set PB2 high to enable motors
    HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    LED3_OFF;
}

void reset_pwm(){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void drive_enable()
{
    mouse.target_angle = angle; //for safety
    mouse.target_dis = get_forward_dis();
    // Set PB2 high to enable motors
    HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET);
    LED3_OFF;
}

void drive_disable()
{
    // Set PB2 low to disable motors
    HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET);
    LED3_OFF;
}

// Function to set differential drive speeds (-1 to 1)
void drive_dif(float left_speed, float right_speed)
{
    // Clamp values to -1.0 to 1.0
    left_speed = fmaxf(fminf(left_speed, 0.8f), -0.8f);
    right_speed = fmaxf(fminf(right_speed, 0.8f), -0.8f);


    static float prev_left = 0.0f;
    static float prev_right = 0.0f;

    
    // Convert to PWM (0 to 4095)
    uint16_t left_pwm = (uint16_t)(fabsf(left_speed) * 4095.0f) % 4096;
    uint16_t right_pwm = (uint16_t)(fabsf(right_speed) * 4095.0f) % 4096;

    static uint32_t last_debug_time = 0;
    uint32_t now = HAL_GetTick();
    if (debug_mot && (now - last_debug_time >= 400)) {
        print("l: %d, r: %d || l: %.2f, r: %.2f\n", left_pwm, right_pwm, left_speed, right_speed);
        last_debug_time = now;
    }

    // Set PWM for left motor (TIM3 CH1 & CH2)

    // Protection: if sign changes, insert a brief neutral (coast) period
    if ((left_speed > 0 && prev_left < 0) || (left_speed < 0 && prev_left > 0)) {
        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 4095);
        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 4095);
        //HAL_Delay(2); // 2 ms coast (tune as needed)
    }
    else if (left_speed < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
    }

    if ((right_speed > 0 && prev_right < 0) || (right_speed < 0 && prev_right > 0)) {
        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 4095);
        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 4095);
        //HAL_Delay(2);
    }
    else if (right_speed < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
    }


    prev_left = left_speed;
    prev_right = right_speed;
}



