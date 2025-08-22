#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx.h"
//#include "stm32f4xx.h"
//#include "delay.h"
#include "led.h"
//#include "button.h"
//#include <stdio.h>
#include "usart.h"
//#include "SPI.h"
//#include "matrixDisplay.h"
//#include "pwm.h"
//#include "encoder.h"
//#include "buzzer.h"
#include "core.h"
#include "sensor_Function.h"
#include "adc.h"

void systick(void);
void button1_interrupt(void);
void button2_interrupt(void);
int core(void);

#endif
