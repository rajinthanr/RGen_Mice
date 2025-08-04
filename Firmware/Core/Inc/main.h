/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define REC_L_Pin GPIO_PIN_0
#define REC_L_GPIO_Port GPIOC
#define REC_FL_Pin GPIO_PIN_1
#define REC_FL_GPIO_Port GPIOC
#define ENC_B1_Pin GPIO_PIN_0
#define ENC_B1_GPIO_Port GPIOA
#define ENC_B2_Pin GPIO_PIN_1
#define ENC_B2_GPIO_Port GPIOA
#define BAT_3V7_Pin GPIO_PIN_2
#define BAT_3V7_GPIO_Port GPIOA
#define BAT_7V4_Pin GPIO_PIN_3
#define BAT_7V4_GPIO_Port GPIOA
#define REC_R_Pin GPIO_PIN_4
#define REC_R_GPIO_Port GPIOA
#define REC_FR_Pin GPIO_PIN_5
#define REC_FR_GPIO_Port GPIOA
#define MOT_A1_Pin GPIO_PIN_6
#define MOT_A1_GPIO_Port GPIOA
#define MOT_A2_Pin GPIO_PIN_7
#define MOT_A2_GPIO_Port GPIOA
#define MOT_B1_Pin GPIO_PIN_0
#define MOT_B1_GPIO_Port GPIOB
#define MOT_B2_Pin GPIO_PIN_1
#define MOT_B2_GPIO_Port GPIOB
#define MOT_ENABLE_Pin GPIO_PIN_2
#define MOT_ENABLE_GPIO_Port GPIOB
#define B_KEY_Pin GPIO_PIN_12
#define B_KEY_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOC
#define ENC_A1_Pin GPIO_PIN_8
#define ENC_A1_GPIO_Port GPIOA
#define ENC_A2_Pin GPIO_PIN_9
#define ENC_A2_GPIO_Port GPIOA
#define TR_FR_Pin GPIO_PIN_11
#define TR_FR_GPIO_Port GPIOA
#define TR_R_Pin GPIO_PIN_12
#define TR_R_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define TR_L_Pin GPIO_PIN_4
#define TR_L_GPIO_Port GPIOB
#define TR_FL_Pin GPIO_PIN_5
#define TR_FL_GPIO_Port GPIOB
#define B_BOOT_Pin GPIO_PIN_8
#define B_BOOT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
