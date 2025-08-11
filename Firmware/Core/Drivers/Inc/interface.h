#ifndef INTERFACE_H
#define INTERFACE_H

#include "main.h"
// Add your interface declarations here
void interface_init();
void LED_On(uint8_t led_num);
void LED_Off(uint8_t led_num);
void LED_Blink(uint8_t led_num, uint32_t delay_ms, uint8_t times);

#endif // INTERFACE_H
