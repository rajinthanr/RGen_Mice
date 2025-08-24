#include "stm32f4xx.h"
#include "main.h"
#include "core.h"

bool is_key_pressed = 0;
bool is_boot_pressed = 0;

void button_Configuration(void) // PA0 flow input
{
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B_BOOT_Pin)
		is_boot_pressed = 1;
	if (GPIO_Pin == B_KEY_Pin)
		is_key_pressed = 0;

	if (is_boot_pressed)
		LED2_ON;
	if (is_key_pressed)
		LED3_ON;
}
