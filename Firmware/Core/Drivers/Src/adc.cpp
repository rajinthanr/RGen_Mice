#include "adc.h"
#include "main.h"

void ADC_Config(void) {}

uint16_t readADC(int8_t pin) {
  ADC_ChannelConfTypeDef sConfig;
  uint16_t adc_val = 0;

  if (pin >= 0 && pin <= 15) {
    switch (pin) {
    case 0:
      sConfig.Channel = ADC_CHANNEL_0;
      break;
    case 1:
      sConfig.Channel = ADC_CHANNEL_1;
      break;
    case 2:
      sConfig.Channel = ADC_CHANNEL_2;
      break;
    case 3:
      sConfig.Channel = ADC_CHANNEL_3;
      break;
    case 4:
      sConfig.Channel = ADC_CHANNEL_4;
      break;
    case 5:
      sConfig.Channel = ADC_CHANNEL_5;
      break;
    case 6:
      sConfig.Channel = ADC_CHANNEL_6;
      break;
    case 7:
      sConfig.Channel = ADC_CHANNEL_7;
      break;
    case 8:
      sConfig.Channel = ADC_CHANNEL_8;
      break;
    case 9:
      sConfig.Channel = ADC_CHANNEL_9;
      break;
    case 10:
      sConfig.Channel = ADC_CHANNEL_10;
      break;
    case 11:
      sConfig.Channel = ADC_CHANNEL_11;
      break;
    case 12:
      sConfig.Channel = ADC_CHANNEL_12;
      break;
    case 13:
      sConfig.Channel = ADC_CHANNEL_13;
      break;
    case 14:
      sConfig.Channel = ADC_CHANNEL_14;
      break;
    case 15:
      sConfig.Channel = ADC_CHANNEL_15;
      break;
    }

    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
      Error_Handler();
    }

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
      adc_val = HAL_ADC_GetValue(&hadc1);
    }
  }

  return adc_val;
}
