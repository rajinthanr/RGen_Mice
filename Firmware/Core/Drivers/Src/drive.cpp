#include "drive.h"
#include "core.h"
#include "encoder.h"
#include "icm.h"
#include "main.h"
#include "math.h"
#include "mouse.h"
#include "sensor.h"
#include "stdbool.h"

extern TIM_HandleTypeDef htim3;

uint8_t debug_mot = 0;
uint8_t is_left_wall = 0;
uint8_t is_right_wall = 0;

// PID controller structure
typedef struct {
  float kp;
  float ki;
  float kd;
  float prev_error;
  float integral;
} PIDController;

// PID controllers for left and right wheels
//                        Kp    Ki    Kd    prev_error  integral
PIDController pid_left = {0.002f, 0.002f, 0.0f, 0.0f, 0.0f};
PIDController pid_right = {0.002f, 0.002f, 0.0f, 0.0f, 0.0f};
PIDController pid_position = {0.4f, 0.3f, 0.1f, 0.0f, 0.0f};
PIDController pid_angle = {0.03f, 0.05f, 2.01f, 0.0f, 0.0f};

#include "delay.h"

// Target speeds (m/s)
static float target_linear = 0.0f;
static float target_angular = 0.0f;
static float output = 0.0f;

float position_controller() {
  static float filtered_error = 0.0f;
  const float alpha = 0.5f; // Low-pass filter coefficient (0 < alpha < 1)

  mouse.target_dis += (mouse.linear_speed + mouse.speed_adj) * LOOP_INTERVAL;
  mouse.speed_adj = 0; // Reset after use

  float error = mouse.target_dis - get_forward_dis();
  filtered_error =
      alpha * error + (1.0f - alpha) * filtered_error; // Low-pass filter
  error = filtered_error;

  float diff = error - pid_position.prev_error;
  pid_position.prev_error = error;
  if (abs(output) < 1.0f)
    pid_position.integral += error * LOOP_INTERVAL; // anti-windup

  pid_position.integral =
      clamp(pid_position.integral, -3.0f, 3.0f); // Anti-windup using clamp

  output = pid_position.kp * error + pid_position.kd * diff +
           pid_position.ki * pid_position.integral;

  return output;
}

float angle_controller() {
  static float output = 0.0f;
  static float filtered_error = 0.0f;
  const float alpha = 0.2f; // Low-pass filter coefficient (0 < alpha < 1)

  mouse.target_angle += mouse.angular_speed * LOOP_INTERVAL;
  if (mouse.steering_mode != STEERING_OFF) {
    wallFollow(is_left_wall, is_right_wall);
    mouse.target_angle += mouse.steering_adjustment * LOOP_INTERVAL;
  }

  if (mouse.is_front_adjust) {
    mouse.wall_error = wallFront();
    mouse.target_angle += mouse.front_adjustment * LOOP_INTERVAL;
  }

  mouse.steering_adjustment = 0; // Reset after use
  mouse.front_adjustment = 0;    // Reset after use

  float error = mouse.target_angle - angle;
  filtered_error = alpha * error + (1.0f - alpha) * filtered_error;
  error = filtered_error;

  float diff = error - pid_angle.prev_error;
  pid_angle.prev_error = error;
  if (abs(output) < 1.0f)
    pid_angle.integral += error * LOOP_INTERVAL;

  pid_angle.integral =
      clamp(pid_angle.integral, -20.0f, 20.0f); // Anti-windup using clamp

  output = pid_angle.kp * error + pid_angle.kd * diff +
           pid_angle.ki * pid_angle.integral;
  return output;
}

void drive_closed_loop_update() {
  float angle_correction = 0;
  float position_correction = 0;

  if (mouse.steering_mode == GYRO_OFF)
    angle_correction = 0;
  else
    angle_correction = angle_controller();
  position_correction = position_controller();

  float output_left = position_correction - angle_correction;
  float output_right = position_correction + angle_correction;

  output_left = clamp(output_left, -1.0f, 1.0f);
  output_right = clamp(output_right, -1.0f, 1.0f);

  // Send to motors
  drive(output_left, output_right);
}

// Set closed-loop targets
void drive_set_closed_loop(float linear, float angular) {
  target_linear = linear;
  target_angular = angular;
  // If speed varies over 100 mm/s, reset everything to zero
  if (fabsf(linear - mouse.linear_speed) > 100.0f) {
    pid_left.integral = 0.0f;
    pid_left.prev_error = 0.0f;
    pid_right.integral = 0.0f;
    pid_right.prev_error = 0.0f;
  }
}

void drive_init() {
  // Set PB2 high to enable motors
  HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  LED3_OFF;
}

void reset_pwm() {
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void drive_enable() {
  mouse.target_angle = angle; // for safety
  mouse.target_dis = get_forward_dis();
  // Set PB2 high to enable motors
  HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_SET);
  LED3_OFF;
}

void drive_disable() {
  // Set PB2 low to disable motors
  HAL_GPIO_WritePin(MOT_ENABLE_GPIO_Port, MOT_ENABLE_Pin, GPIO_PIN_RESET);
  LED3_OFF;
}

// Function to set differential drive speeds (-1 to 1)
void drive(float left_speed, float right_speed) {
  // Clamp values to -1.0 to 1.0
  left_speed = clamp(left_speed, -1.0f, 1.0f) * 0.8f;
  right_speed = clamp(right_speed, -1.0f, 1.0f) * 0.8f;

  static float prev_left = 0.0f;
  static float prev_right = 0.0f;

  // Convert to PWM (0 to 4095)
  uint16_t left_pwm = (uint16_t)(fabsf(left_speed) * 4095.0f) % 4096;
  uint16_t right_pwm = (uint16_t)(fabsf(right_speed) * 4095.0f) % 4096;

  static uint32_t last_debug_time = 0;
  uint32_t now = HAL_GetTick();
  if (debug_mot && (now - last_debug_time >= 400)) {
    print("l: %d, r: %d || l: %.2f, r: %.2f\n", left_pwm, right_pwm, left_speed,
          right_speed);
    last_debug_time = now;
  }

  // Set PWM for left motor (TIM3 CH1 & CH2)

  // Protection: if sign changes, insert a brief neutral (coast) period
  if ((left_speed > 0 && prev_left < 0) || (left_speed < 0 && prev_left > 0)) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 4095);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 4095);
    // HAL_Delay(2); // 2 ms coast (tune as needed)
  } else if (left_speed < 0) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
  }

  if ((right_speed > 0 && prev_right < 0) ||
      (right_speed < 0 && prev_right > 0)) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 4095);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 4095);
    // HAL_Delay(2);
  } else if (right_speed < 0) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  } else {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
  }
  prev_left = left_speed;
  prev_right = right_speed;
}
