#ifndef SENSOR_FUNCTION_H
#define SENSOR_FUNCTION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stm32f4xx.h>
#include "main.h"
#include "stdint.h"

    extern int reflectionRate;
    extern float cell_1;
    extern float cell_2;

    extern int32_t volMeter;
    extern int32_t voltage;
    extern int32_t LSensor;
    extern int32_t RSensor;
    extern int32_t FLSensor;
    extern int32_t FRSensor;
    extern int32_t Outz;
    extern float aSpeed;
    extern float angle;

    extern uint8_t NO_START;
    extern uint8_t LEFT_START;
    extern uint8_t RIGHT_START;

    extern int reading[4];
    extern float dis_reading[4];
    extern uint8_t is_sensor_active;

    enum {
  STEER_NORMAL,
  STEER_LEFT_WALL,
  STEER_RIGHT_WALL,
  STEERING_OFF,
  GYRO_OFF
  };

// Re-order to match logical order: FL, L, R, FR -> channels: 11,10,4,5
#define read_L_Sensor readADC(10)
#define read_R_Sensor readADC(4)
#define read_FL_Sensor readADC(11)
#define read_FR_Sensor readADC(5)

#define R_EM_ON HAL_GPIO_WritePin(TR_R_GPIO_Port, TR_R_Pin, GPIO_PIN_SET);
#define R_EM_OFF HAL_GPIO_WritePin(TR_R_GPIO_Port, TR_R_Pin, GPIO_PIN_RESET);
#define L_EM_ON HAL_GPIO_WritePin(TR_L_GPIO_Port, TR_L_Pin, GPIO_PIN_SET);
#define L_EM_OFF HAL_GPIO_WritePin(TR_L_GPIO_Port, TR_L_Pin, GPIO_PIN_RESET);
#define FR_EM_ON HAL_GPIO_WritePin(TR_FR_GPIO_Port, TR_FR_Pin, GPIO_PIN_SET);
#define FR_EM_OFF HAL_GPIO_WritePin(TR_FR_GPIO_Port, TR_FR_Pin, GPIO_PIN_RESET);
#define FL_EM_ON HAL_GPIO_WritePin(TR_FL_GPIO_Port, TR_FL_Pin, GPIO_PIN_SET);
#define FL_EM_OFF HAL_GPIO_WritePin(TR_FL_GPIO_Port, TR_FL_Pin, GPIO_PIN_RESET);

    float dist(int ir_num);
    bool is_wall(int w);
    void readSensor(void);
    float get_front_sum();
    void readGyro(void);
    void readVolMeter(void);
    void lowBatCheck(void);
    void IR_Configuration(void);
    void enable();
    void disable();
    uint8_t occluded_left();
    uint8_t occluded_right();
    uint8_t wait_for_user_start();
    void set_steering_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif
