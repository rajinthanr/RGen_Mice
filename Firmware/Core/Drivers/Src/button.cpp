#include "stm32f4xx.h"
#include "main.h"
#include"core.h"

void button_Configuration(void)//PA0 flow input
{
		
		
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == B_BOOT_Pin) LED2_ON;
	if(GPIO_Pin == B_KEY_Pin) LED2_OFF;
}
