#ifndef LED_H
#define LED_H

#include "stm32f4xx.h"

#define LED1_ON    LED_On(1)
#define LED1_OFF   LED_Off(1)
#define LED2_ON    LED_On(2)
#define LED2_OFF   LED_Off(2)
#define LED3_ON    LED_On(3)
#define LED3_OFF   LED_Off(3)
#define LED4_ON    LED_On(4)
#define LED4_OFF   LED_Off(4)

#define RF_EM_ON   GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define RF_EM_OFF  GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define LF_EM_ON   GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define LF_EM_OFF  GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define SIDE_EM_ON   GPIO_SetBits(GPIOA, GPIO_Pin_7)
#define SIDE_EM_OFF  GPIO_ResetBits(GPIOA, GPIO_Pin_7)

#define ALL_LED_OFF LED1_OFF; \
                    LED2_OFF; \
                    LED3_OFF; \
					LED4_OFF; \
										
#define ALL_LED_ON  LED1_ON; \
                    LED2_ON; \
					LED3_ON; \
					LED4_ON; \

void LED_Configuration(void);

#endif /* LED_H */
