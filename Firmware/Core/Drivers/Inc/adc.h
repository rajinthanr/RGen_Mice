#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
#include "core.h"
#include "main.h"
#include "stdint.h"
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;

void ADC_Config(void);
uint16_t readADC(int8_t pin);

#ifdef __cplusplus
}
#endif

#endif
