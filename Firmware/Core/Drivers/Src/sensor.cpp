// #include "global.h"
#include "sensor.h"
#include "adc.h"
#include "core.h"
#include "delay.h"
#include "encoder.h"
#include "icm.h"
#include "led.h"
#include "main.h"

float cell_1 = 0;
float cell_2 = 0;

int32_t LSensor = 0;
int32_t RSensor = 0;
int32_t FLSensor = 0;
int32_t FRSensor = 0;
int32_t Outz = 0;
float aSpeed = 0; // angular velocity
float angle = 0;
int reading[4];
float dis_reading[4];

uint8_t NO_START = 0;
uint8_t LEFT_START = 1;
uint8_t RIGHT_START = 2;
uint8_t is_collision_detection = 1;
uint8_t is_collided = 0;

int max_difference[4];

float dist(int ir_num) {
  if (reading[ir_num] < 10)
    return 1000;

  else {
    float ratio = 1.0 * initial_wall[ir_num] / reading[ir_num];
    if (ratio >= 0) {
      float distance = 0;
      if (ir_num == L || ir_num == R)
        distance = SIDE_WALL_DISTANCE_CAL * sqrt(ratio);
      else
        distance = FRONT_WALL_DISTANCE_CAL * sqrt(ratio);

      return distance;
    } else {
      return 9000;
    }
  }
}

bool is_wall(int w) {
  if (w == FL || w == FR)
    return (get_front_dis() <= 400);
  return dis_reading[w] <= 150;
}

void set_steering_mode(uint8_t mode) {
  if (mouse.steering_mode == GYRO_OFF) {
    angle = mouse.target_angle;
  }
  mouse.last_steering_error = 0;
  mouse.steering_adjustment = 0;
  mouse.steering_mode = mode;
}

/*read IR sensors*/
void readSensor(void) {
  // Take 'average' readings and compute the average for each sensor
  int avg_count = 1; // You can change this value or pass as a parameter
  uint16_t exposure_time = 80; // in microseconds

  uint32_t curt = micros();

  int l_sum = 0, r_sum = 0, fl_sum = 0, fr_sum = 0;
  for (int i = 0; i < avg_count; ++i) {
    l_sum += read_L_Sensor;
    r_sum += read_R_Sensor;
    fl_sum += read_FL_Sensor;
    fr_sum += read_FR_Sensor;
  }
  LSensor = l_sum / avg_count;
  RSensor = r_sum / avg_count;
  FLSensor = fl_sum / avg_count;
  FRSensor = fr_sum / avg_count;

  // Front wall sensors first
  FL_EM_ON;
  elapseMicros(exposure_time, curt);
  int fl_em_sum = 0;
  for (int i = 0; i < avg_count; ++i)
    fl_em_sum += read_FL_Sensor;
  FLSensor = (fl_em_sum / avg_count) - FLSensor;
  FL_EM_OFF;

  FR_EM_ON;
  elapseMicros(exposure_time * 2, curt);
  int fr_em_sum = 0;
  for (int i = 0; i < avg_count; ++i)
    fr_em_sum += read_FR_Sensor;
  FRSensor = (fr_em_sum / avg_count) - FRSensor;
  FR_EM_OFF;

  // Then the side wall sensors
  // if (!mouse.is_front_adjust)
  { // If front wall adjustment needed, skip side
    // sensors to save time
    L_EM_ON;
    elapseMicros(exposure_time * 3, curt);
    int l_em_sum = 0;
    for (int i = 0; i < avg_count; ++i)
      l_em_sum += read_L_Sensor;
    LSensor = (l_em_sum / avg_count) - LSensor;
    L_EM_OFF;

    R_EM_ON;
    elapseMicros(exposure_time * 4, curt);
    int r_em_sum = 0;
    for (int i = 0; i < avg_count; ++i)
      r_em_sum += read_R_Sensor;
    RSensor = (r_em_sum / avg_count) - RSensor;
    R_EM_OFF;
  }

  if (FLSensor < 0)
    FLSensor = 0;
  if (FRSensor < 0)
    FRSensor = 0;
  if (LSensor < 0)
    LSensor = 0;
  if (RSensor < 0)
    RSensor = 0;

  // // readVolMeter();
  // static int sensor_history[4][100];
  // static int history_index = 0;

  // // Store the current readings for all 4 sensors
  // sensor_history[0][history_index] = LSensor;
  // sensor_history[1][history_index] = FLSensor;
  // sensor_history[2][history_index] = FRSensor;
  // sensor_history[3][history_index] = RSensor;

  // // Find max difference for each sensor
  // max_difference[4] = {0};
  // for (int s = 0; s < 4; s++) {
  //   int max_val = sensor_history[s][1];
  //   int min_val = sensor_history[s][1];
  //   for (int i = 1; i < 100; i++) {
  //     if (sensor_history[s][i] > max_val)
  //       max_val = sensor_history[s][i];
  //     if (sensor_history[s][i] < min_val)
  //       min_val = sensor_history[s][i];
  //   }
  //   max_difference[s] = max_val - min_val;
  // }
  // history_index = (history_index + 1) % 100;

  reading[0] = LSensor;
  reading[1] = FLSensor;
  reading[2] = FRSensor;
  reading[3] = RSensor;

  for (int i = 0; i < 4; i++) {
    dis_reading[i] = dist(i);
  }
  dis_reading[FL] += FRONT_SENSOR_DISPLACEMENT;
  dis_reading[FR] += FRONT_SENSOR_DISPLACEMENT;
  dis_reading[L] += SIDE_SENSOR_SPACING / 2;
  dis_reading[R] += SIDE_SENSOR_SPACING / 2;
}
// there are 1000 - 320 = 680 us remaining in a 1ms_ISR

float get_front_dis() { return (dis_reading[FL] + dis_reading[FR]) / 2; }

/*read gyro*/
void readGyro(void) { // k=19791(sum for sample in 1 second)    101376287 for 50
                      // seconds with 5000 samples
  aSpeed = get_gyroZ();

  float LOOP_INTERVAL = 0.001;
  angle += aSpeed * LOOP_INTERVAL;
}

void safety_stop(int duration = 100) {
  is_mouse_enable = 0;
  drive_disable();
  LEDS_OFF;

  while (1) {
    LEDS_OFF;
    delay_ms(duration * 4);

    LED3_ON;
    delay_ms(duration);
  }
}

void collisionDetection(void) {
  if (!is_collision_detection)
    return;
  static float accY = 0;
  // accY += get_accY() * 0.1 + accY * 0.9;
  // if (accY < -40) // If acceleration in Y direction exceeds 30 m/s^2]
  if (abs(mouse.target_angle - angle) > 10) {
    is_mouse_enable = 0;
    drive_disable();
    is_collided = 1;
    print("Collision detected!\n");
    LED3_ON;
  }
}

/*read voltage meter*/
void readVolMeter(void) {
  float c3_7 = readADC(2);
  float v7_4 = readADC(3);

  c3_7 = c3_7 * 0.488 * 3.3;
  v7_4 = v7_4 * 0.731 * 3.3;

  cell_1 = c3_7;
  cell_2 = v7_4 - c3_7;
}

void lowBatCheck(void) {
  if (cell_1 < 3500 ||
      cell_2 < 3500) // alert when battery Voltage lower than 7V
  {
    print("Low battery detected!");
    safety_stop();
  }
}

void IR_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = TR_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TR_L_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TR_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TR_R_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TR_FL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TR_FL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TR_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TR_FR_GPIO_Port, &GPIO_InitStruct);
}

uint8_t occluded_left() {
  return dis_reading[FL] < 100 && dis_reading[FR] > 100;
}

uint8_t occluded_right() {
  return dis_reading[FL] > 100 && dis_reading[FR] < 100;
}

float clamp(float value, float min, float max) {
  if (value < min)
    return min;
  else if (value > max)
    return max;
  return value;
}
