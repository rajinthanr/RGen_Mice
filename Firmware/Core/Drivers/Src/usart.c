#include "stm32f4xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1; // Change to your UART instance



int _read(int file, char *ptr, int len)
{
  HAL_UART_Receive(&huart1, (uint8_t *)ptr, 1, HAL_MAX_DELAY);
  return 1;
}
