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
    //if(Millis<2000) return;
    
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
    LED2_ON;
    print("initialing..\r\n");
    delay_ms(100);

    UART_Configurations();
    init_flash();
    Wall_Configuration();
    LED_Configuration();
    IR_Configuration();
    Encoder_Configration();
    ADC_Config();
    
    maze.initialise();
    drive_init();
    print("Core initialized\r\n");

    if(switches.key_pressed()){
        LED1_ON;
        LED4_ON;
        print("Boot button pressed, calibration mode\r\n");
        is_calibrate = 1;
        while(switches.boot_pressed());
        delay_ms(1000);
        LEDS_OFF;
        cal_initial_wall();
    }

    is_sensor_active = true;
    maze.set_goal(Location(7, 7)); //default goal

    Systick_Configuration();

    if(switches.boot_pressed()){
        
    }

    

    while (1)
    {
        if(switches.key_pressed()){
            uint8_t is_decided = 0;
            LED4_ON;
            drive_enable();
            is_mouse_enable = 1;
            delay_ms(1000);
            while(!is_decided){
                if(occluded_left()){
                    print("Left start selected\r\n");
                    is_decided = 1;
                    maze.set_goal(Location(3, 2));
                    while(occluded_left());
                }
                else if(occluded_right()){is_decided = 1;}
            }
            print("Start in 2 seconds\r\n");
            LEDS_ON;
            delay_ms(500);
            LED4_OFF;
            delay_ms(500);
            LED3_OFF;
            delay_ms(500);
            LED2_OFF;
            delay_ms(500);
            LED1_OFF;
            icm_initialize();
            print("Search started\n");
            mouse.search(maze.goal());
            is_mouse_enable = 0;
            drive_disable();
            maze.save_to_flash();
            maze.set_goal(Location(0, 0));
            drive_enable();
            is_mouse_enable = 1;
            mouse.search(maze.goal());
            is_mouse_enable = 0;
            drive_disable();
            maze.save_to_flash();
        }else{
            
        }

        if(switches.boot_pressed()){
            LED2_ON;
            print("Boot button pressed, fast run mode\r\n");
            while(switches.boot_pressed());
            delay_ms(1000);
            LED2_OFF;
            delay_ms(1000);
            icm_initialize();
            maze.load_from_flash();
            maze.set_goal(Location(3, 2));
            is_mouse_enable = 1;
            drive_enable();
            mouse.search(maze.goal());
            is_mouse_enable = 0;
            drive_disable();
            maze.set_goal(Location(0, 0));
            drive_enable();
            is_mouse_enable = 1;
            mouse.search(maze.goal());
            is_mouse_enable = 0;
            drive_disable();
        }else{
            
        }

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
            icm_initialize();
            //static int8_t state = -1;
            //drive_dif(state,state);
            //state = -state;
            //mouse.search_maze();
            //motion.spin_turn(720, mouse.max_angular_speed, mouse.max_angular_accel);
            print("Search started\n");
            mouse.search(maze.goal());
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
