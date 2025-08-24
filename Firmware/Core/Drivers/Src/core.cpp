
#include "core.h"
#include "motion.h"
#include "config.h"
#include "sensors.h"
#include "mouse.h"

extern UART_HandleTypeDef huart1; // Change to your UART instance
uint8_t is_run = 0;

Motion motion;    // high level motion operations
Motors motors;    // low level control for drive motors
Profile forward;  // speed profiles for forward motion
Profile rotation; // speed profiles for rotary motion
Sensors sensors;
Mouse mouse;

bool systick_enable = 0;

float left_measured;
float right_measured;
bool is_mouse_enable = 0;
bool is_wall_follow = 0;

// Retarget printf to UART

void systick(void)
{
	if(systick_enable){
    update();
    motion.update();
    motors.update_controllers(motion.velocity(), motion.omega(), sensors.get_steering_feedback());
	}
}

int core(void)
{
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
    while (1)
    {
        LED1_ON;
        delay_ms(10);
        // readSensor();
        //   read_values();
        readSensor();
        //readGyro();
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
            // drive_set_closed_loop(mice.max_linear_speed, mice.max_angular_speed);
            mouse.turn_IP180();
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
        if (is_mouse_enable)
        {
        	systick_enable=1;
            //drive_closed_loop_update();
            // drive_dif(mice.max_linear_speed/100,mice.max_linear_speed/100);
        }
        if (is_wall_follow)
        {
            wallFollow(true, true);
        }
    }
    return 0;
}
