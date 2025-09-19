#include "led.h"
#include "main.h"

void LED_Configuration(void) {
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		// Configure LED Pin as output
	    GPIO_InitStruct.Pin = LED1_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	    // Configure LED2 Pin as output
	    GPIO_InitStruct.Pin = LED2_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

	    // Configure LED3 Pin as output
	    GPIO_InitStruct.Pin = LED3_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

	    // Configure LED4 Pin as output
	    GPIO_InitStruct.Pin = LED4_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

	    // Configure Button Pin as input
	    GPIO_InitStruct.Pin = B_BOOT_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(B_BOOT_GPIO_Port, &GPIO_InitStruct);

	    // Configure B_KEY Pin as input
	    GPIO_InitStruct.Pin = B_KEY_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(B_KEY_GPIO_Port, &GPIO_InitStruct);
}


typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} LedConfig;

// Array of LED configurations (adjust as needed)
static const LedConfig led_configs[] = {
    {LED1_GPIO_Port, LED1_Pin},
    {LED2_GPIO_Port, LED2_Pin},
    {LED3_GPIO_Port, LED3_Pin},
    {LED4_GPIO_Port, LED4_Pin}
};

void LED_On(uint8_t led_num)
{
    if (led_num == 0 || led_num > sizeof(led_configs)/sizeof(led_configs[0]))
        return;
    HAL_GPIO_WritePin(led_configs[led_num-1].port, led_configs[led_num-1].pin, GPIO_PIN_SET);
}

void LED_Off(uint8_t led_num)
{
    if (led_num == 0 || led_num > sizeof(led_configs)/sizeof(led_configs[0]))
        return;
    HAL_GPIO_WritePin(led_configs[led_num-1].port, led_configs[led_num-1].pin, GPIO_PIN_RESET);
}

void LED_Blink(uint8_t led_num, uint32_t delay_time, uint8_t times)
{
    if (led_num == 0 || led_num > sizeof(led_configs)/sizeof(led_configs[0]))
        return;
    for (uint8_t i = 0; i < times; i++)
    {
    HAL_GPIO_WritePin(led_configs[led_num-1].port, led_configs[led_num-1].pin, GPIO_PIN_SET);
        delay_ms(delay_time);
    HAL_GPIO_WritePin(led_configs[led_num-1].port, led_configs[led_num-1].pin, GPIO_PIN_RESET);
    	delay_ms(delay_time);
    }
}
