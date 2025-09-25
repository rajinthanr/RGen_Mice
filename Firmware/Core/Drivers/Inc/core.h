#ifndef CORE_H
#define CORE_H


#ifdef __cplusplus
#include <stdio.h>
#include <string>
#include <sstream>

#include "config.h"
#include "motion.h"
#include "profile.h"
#include "mouse.h"
#include "delay.h"
#include "usart.h"
#include "maze.h"
#include "led.h"
#include "wall_handle.h"
#include "flash.h"
#include "drive.h"
#include "icm.h"
#include "sensor_Function.h"
#include "encoder.h"
#include "adc.h"

#include <cstring>
#include "button.h"
extern "C"
{
#endif

#include "main.h"
#include "stdio.h"
#include "stm32f4xx.h"


    extern uint8_t is_run;
    extern uint8_t is_mouse_enable;
    extern uint8_t is_wall_follow;

    void systick(void);
    void button1_interrupt(void);
    void button2_interrupt(void);
    int core(void);

#ifdef __cplusplus
}
#endif

#endif
