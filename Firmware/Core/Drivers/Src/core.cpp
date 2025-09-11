#include "core.h"
#include "main.h"
#include "delay.h"
#include "adc.h"
#include "led.h"
#include "sensor_Function.h"
#include "encoder.h"
#include "icm.h"
#include "drive.h"
#include "usart.h"
#include <cstring>
#include "flash.h"
#include "wall_handle.h"


Motion motion;                            // high level motion operations
Profile forward;                          // speed profiles for forward motion
Profile rotation;                         // speed profiles for rotary motion

extern UART_HandleTypeDef huart1; // Change to your UART instance
uint8_t is_run = 0;

float left_measured;
float right_measured;
bool is_mouse_enable = 0;
bool is_wall_follow = 0;

// Retarget printf to UART

void systick(void)
{
    if(Millis<1000) return;
    readGyro();
    if (is_mouse_enable)
        {
            mouse.linear_speed = motion.velocity();
            mouse.angular_speed = motion.omega();
            drive_closed_loop_update();
            motion.update();
            // drive_dif(mouse.max_linear_speed/100,mouse.max_linear_speed/100);
        }
}
int core(void)
{
    printf("initialing..\r\n");
    delay_ms(50);
    init_flash();
    Wall_Configuration();
    Systick_Configuration();
    LED_Configuration();
    IR_Configuration();
    icm_initialize();
    UART_Configurations();
    // drive(0.6,0);
    //     button_Configuration();
    //     usart1_Configuration(9600);
    //     SPI_Configuration();
    //     TIM4_PWM_Init();
    Encoder_Configration();
    //    buzzer_Configuration();
    ADC_Config();
    //
    //    shortBeep(2000, 8000);
    printf("Core initialized\r\n");
    while (1)
    {
        LED1_ON;
        delay_ms(1);
        readSensor();
        //  read_values();
        //readSensor();
        readVolMeter();
        static uint32_t lastTick = 0;
        if (HAL_GetTick() - lastTick >= 500)
        { // 500ms = 2 times per second
            lowBatCheck();
            lastTick = HAL_GetTick();
            // printf("L %d R %d FL %d FR %d aSpeed %.2f angle %.2f voltage %d lenc %d renc %d\r\n", LSensor, RSensor, FLSensor, FRSensor, get_gyroZ(), angle, voltage, getLeftEncCount(), getRightEncCount());
        }
        //        printf("LF %d RF %d DL %d DR %d aSpeed %d angle %d voltage %d lenc %d renc %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor, aSpeed, angle, voltage, getLeftEncCount(), getRightEncCount());
        //        displayMatrix("mous");
        //
        //        setLeftPwm(100);
        //        setRightPwm(100);
        //        delay_ms(1000);
        if (is_run)
        {
            motion.spin_turn(720, mouse.max_angular_speed, mouse.max_angular_accel);
            is_run = 0;
        }
        if (is_calibrate)
        {
            cal_initial_wall();
            is_calibrate = 0;
        }
        if (is_wall_front)
        {
            wallFront(0);
        }
        
        if (is_wall_follow)
        {
            wallFollow(true, true);
        }
    }
    return 0;
}
