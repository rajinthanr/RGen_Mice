#ifndef CORE_H
#define CORE_H

#ifdef __cplusplus
#include <stdio.h>
#include <string>
#include <sstream>
#include "motors.h"

#include "main.h"
#include "delay.h"
#include "adc.h"
#include "led.h"
#include "sensor_Function.h"
#include "encoder.h"
#include "icm.h"
#include "drive.h"
#include "usart.h"
#include "flash.h"
#include "wall_handle.h"
extern "C"
{
#endif

#include "stdio.h"
#include "stm32f4xx.h"

#include "main.h"
#include "delay.h"
#include "adc.h"
#include "led.h"
#include "sensor_Function.h"
#include "encoder.h"
#include "icm.h"
#include "drive.h"
#include "usart.h"
#include "flash.h"
#include "wall_handle.h"

#include <stdio.h>

    extern uint8_t is_run;
    extern bool is_mouse_enable;
    extern bool is_wall_follow;

    void systick(void);
    void button1_interrupt(void);
    void button2_interrupt(void);
    int core(void);

#ifdef __cplusplus
}
#endif

#endif
