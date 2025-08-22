#ifndef DELAY_H
#define DELAY_H   
#include "stdbool.h" //for bool
#include "main.h"

void Systick_Configuration(void);//initialize systick
void delay_ms(uint32_t nTime);
void delay_us(uint32_t nTime);
uint32_t micros(void);
uint32_t millis(void);
void elapseMicros(uint32_t targetTime, uint32_t oldt);
void elapseMillis(uint32_t targetTime, uint32_t oldt);

extern volatile uint32_t Millis;
extern volatile uint32_t Micros;

#define useExternalOSC 0     //true=1/false=0
#define systemFrequency 168   //MHz


#endif
