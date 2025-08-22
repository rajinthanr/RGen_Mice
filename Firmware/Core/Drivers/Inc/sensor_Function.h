#ifndef SENSOR_FUNCTION_H
#define SENSOR_FUNCTION_H 

#include <stm32f4xx.h>
#include "main.h"

extern int reflectionRate;

extern int32_t volMeter;
extern int32_t voltage;
extern int32_t LSensor;
extern int32_t RSensor;
extern int32_t FLSensor;
extern int32_t FRSensor;
extern int32_t Outz;
extern int32_t aSpeed;
extern int32_t angle;

// Re-order to match logical order: FL, L, R, FR -> channels: 11,10,4,5
#define read_L_Sensor      readADC(10)
#define read_R_Sensor      readADC(4)
#define	read_FL_Sensor     readADC(11)
#define read_FR_Sensor     readADC(5)



#define R_EM_ON    HAL_GPIO_WritePin(TR_R_GPIO_Port, TR_R_Pin, GPIO_PIN_SET);
#define R_EM_OFF   HAL_GPIO_WritePin(TR_R_GPIO_Port, TR_R_Pin, GPIO_PIN_RESET);
#define L_EM_ON    HAL_GPIO_WritePin(TR_L_GPIO_Port, TR_L_Pin, GPIO_PIN_SET);
#define L_EM_OFF   HAL_GPIO_WritePin(TR_L_GPIO_Port, TR_L_Pin, GPIO_PIN_RESET);
#define FR_EM_ON   HAL_GPIO_WritePin(TR_FR_GPIO_Port, TR_FR_Pin, GPIO_PIN_SET);
#define FR_EM_OFF  HAL_GPIO_WritePin(TR_FR_GPIO_Port, TR_FR_Pin, GPIO_PIN_RESET);
#define FL_EM_ON   HAL_GPIO_WritePin(TR_FL_GPIO_Port, TR_FL_Pin, GPIO_PIN_SET);
#define FL_EM_OFF  HAL_GPIO_WritePin(TR_FL_GPIO_Port, TR_FL_Pin, GPIO_PIN_RESET);

void readSensor(void);
void readGyro(void);
void readVolMeter(void);
void lowBatCheck(void);
void IR_Configuration(void);

#endif
