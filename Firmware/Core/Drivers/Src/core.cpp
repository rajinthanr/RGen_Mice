#include "core.h"


Motion motion;                            // high level motion operations
Profile forward;                          // speed profiles for forward motion
Profile rotation;                         // speed profiles for rotary motion
Switches switches;
Mouse mouse;
Maze maze;

extern UART_HandleTypeDef huart1; // Change to your UART instance
uint8_t is_run = 0;

float left_measured;
float right_measured;
uint8_t is_mouse_enable = 0;
uint8_t is_wall_follow = 0;

// Retarget print to UART

float theta;

void systick(void)
{
    if(Millis<2000) return;
    
    //HAL_UART_TxCpltCallback(&huart1);

    if (is_mouse_enable)
        {
            readGyro();
            readSensor();
            mouse.linear_speed = motion.velocity();
            mouse.angular_speed = motion.omega();
            motion.update();
            drive_closed_loop_update();
            // drive_dif(mouse.max_linear_speed/100,mouse.max_linear_speed/100);
        }
}


int core(void)
{
    print("initialing..\r\n");
   // delay_ms(50);
    init_flash();
    Wall_Configuration();
    Systick_Configuration();
    LED_Configuration();
    IR_Configuration();
    icm_initialize();
    UART_Configurations();
    
    Encoder_Configration();
    ADC_Config();
    maze.initialise();
    drive_init();
    print("Core initialized\r\n");
    is_sensor_active = true;

    while (1)
    {
        delay_ms(1);
        readVolMeter();
        static uint32_t lastTick = 0;
        if (HAL_GetTick() - lastTick >= 500)
        { // 500ms = 2 times per second
            LED2_TOGGLE;
            lowBatCheck();
            lastTick = HAL_GetTick();
            // print("L %d R %d FL %d FR %d aSpeed %.2f angle %.2f voltage %d lenc %d renc %d\r\n", LSensor, RSensor, FLSensor, FRSensor, get_gyroZ(), angle, voltage, getLeftEncCount(), getRightEncCount());
        }
        

        if (is_run)
        {
            //static int8_t state = -1;
            //drive_dif(state,state);
            //state = -state;
            //mouse.search_maze();
            //motion.spin_turn(720, mouse.max_angular_speed, mouse.max_angular_accel);
            print("Search started\n");
            mouse.search_to(maze.goal());
            maze.flood(START);
            is_run = 0;
        }

        if (is_calibrate)
        {
            cal_initial_wall();
            is_calibrate = 0;
        }
        // if (is_wall_front)
        // {
        //     wallFront();
        // }
        
        // if (is_wall_follow)
        // {
        //     wallFollow(true, true);
        // }
    }
    return 0;
}
