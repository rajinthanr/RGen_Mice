#ifndef CORE_H
#define CORE_H

#ifdef __cplusplus
#include <sstream>
#include <stdio.h>
#include <string>

#include "adc.h"
#include "config.h"
#include "delay.h"
#include "drive.h"
#include "encoder.h"
#include "flash.h"
#include "icm.h"
#include "led.h"
#include "maze.h"
#include "motion.h"
#include "mouse.h"
#include "profile.h"
#include "sensor.h"
#include "usart.h"
#include "wall_handle.h"

#include "button.h"
#include <cstring>
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
#include "stm32f4xx.h"


extern uint8_t is_run;
extern uint8_t is_mouse_enable;
extern uint8_t is_wall_follow;
extern uint8_t is_icm_init;

void systick(void);
int core(void);

#ifdef __cplusplus
}
#endif

#endif
