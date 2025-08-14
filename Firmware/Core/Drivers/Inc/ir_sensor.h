#ifndef IR_SENSORS_H
#define IR_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

	extern ADC_HandleTypeDef hadc1;
	extern TIM_HandleTypeDef htim10;
	extern int reading[4];
	extern float dis_reading[4];

	// Public API
	void ir_emitor(uint32_t idx, bool state);
	void update_ir(void);
	float map(float value, float fromLow, float fromHigh, float toLow, float toHigh);
	void print_sensor_readings(void);
	uint16_t readADC(int8_t pin);
	void delay(uint32_t ms);
	void delayMicroseconds(uint32_t us);
	uint64_t millis(void);
	uint64_t micros(void);

#ifdef __cplusplus
}
#endif

#endif // IR_SENSORS_H