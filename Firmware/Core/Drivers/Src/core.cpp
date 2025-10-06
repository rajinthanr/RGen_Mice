#define DEBUG 1

#include "core.h"

Motion motion;    // high level motion operations
Profile forward;  // speed profiles for forward motion
Profile rotation; // speed profiles for rotary motion
Switches switches;
Mouse mouse;
Maze maze;

uint8_t is_run = 0;
uint8_t is_mouse_enable = 0;
uint8_t is_wall_follow = 0;
uint8_t is_icm_init = 0;

//****************** User Configurable Parameters ***************** */
Location GOAL(7, 7); // default goal location ********************
Location HOME(0, 0);

void systick(void) {
  // acc_y = get_accY();
  if (is_mouse_enable) {
    readGyro();
    readSensor();
    collisionDetection();
    readVolMeter();
    mouse.linear_speed = motion.velocity();
    mouse.angular_speed = motion.omega();
    motion.update();
    drive_closed_loop_update();
  }
}

int core(void) {
  LED2_ON;
  print("initialing..\r\n");

  Systick_Configuration();
  delay_ms(100);

  UART_Configurations();
  init_flash();
  Wall_Configuration();
  LED_Configuration();
  IR_Configuration();
  Encoder_Configration();
  ADC_Config();
  readVolMeter();

  maze.initialise();
  maze.set_goal(GOAL);
  drive_init();

  print("Core initialized\r\n");

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // maze.set_goal(GOAL);
  // mouse.plan_diagonal(maze.goal());
  // mouse.print_plan();

  if (switches.key_pressed()) {
    LED1_ON;
    print("Boot button pressed, calibration mode\r\n");
    while (switches.boot_pressed())
      ;
    delay_ms(1000);
    LEDS_OFF;
    cal_initial_wall();
    delay_ms(100);
    NVIC_SystemReset();
  }

  //*****************  Main loop  ***************** */

  while (1) {

    if (switches.boot_pressed()) { // Start button pressed - initiate search run
                                   // or fast run
      uint8_t is_decided = 0;
      LED2_ON;
      is_mouse_enable = 1;
      delay_ms(1000);

      while (!is_decided) {
        if (occluded_left()) {
          LED1_ON;
          print("Left start selected - search run\n");
          is_decided = 1;

        } else if (occluded_right()) {
          LED4_ON;
          print("Right start selected - fast run\n");
          is_decided = 2;
        } else if (switches.key_pressed()) {
          LED_Blink(2, 100, 3);
          print("Optimized run selected\n");
          while (switches.key_pressed())
            ;
          delay_ms(20);
          is_decided = 3;
        } else if (switches.boot_pressed()) {
          LED_Blink(3, 100, 3);
          print("Optimized diagonal run selected\n");
          while (switches.boot_pressed())
            ;
          delay_ms(20);
          is_decided = 4;
        }
      }

      maze.set_goal(GOAL);
      print("Start in 2 seconds\r\n");
      LEDS_ON;
      delay_ms(500);
      LED4_OFF;
      delay_ms(500);
      LED3_OFF;
      delay_ms(500);
      LED2_OFF;
      icm_initialize();
      LED1_OFF;
      print("Search started\n");
      drive_enable();

      if (is_decided == 1) {
        mouse.home_run();
        uint8_t is_hit_target = mouse.search(maze.goal());
        is_mouse_enable = 0;
        drive_disable();
        if (is_hit_target)
          maze.save_to_flash();
        maze.set_goal(HOME);
        drive_enable();
        is_mouse_enable = 1;
        is_hit_target = mouse.search(maze.goal());
        is_mouse_enable = 0;
        drive_disable();
        if (is_hit_target)
          maze.save_to_flash();
      }

      else if (is_decided == 2) {
        maze.load_from_flash();
        mouse.home_run();
        if (mouse.fast_run(maze.goal()))
          mouse.fast_run(Location(0, 0));
        is_mouse_enable = 0;
        drive_disable();
      }

      else if (is_decided == 3) {
        mouse.plan(maze.goal());
        mouse.print_plan();
        mouse.home_run();
        if (mouse.start()) {
          mouse.is_smooth_turn = 0; // return with stability
          mouse.smooth_only = 0;
          mouse.plan(HOME);
          mouse.print_plan();
          mouse.start();
        }
        is_mouse_enable = 0;
        drive_disable();
      }

      else if (is_decided == 4) {
        mouse.plan_diagonal(maze.goal());
        mouse.print_plan();
        mouse.home_run();
        if (mouse.start_diagonal()) {
          mouse.is_smooth_turn = 0; // return with stability
          mouse.smooth_only = 0;
          mouse.plan(HOME);
          mouse.print_plan();
          mouse.start();
        }
        is_mouse_enable = 0;
        drive_disable();
      }
    }

    if (switches.key_pressed()) { // Allows user to configure settings

      LED4_ON;
      while (switches.key_pressed())
        ;
      LED4_OFF;
      delay_ms(10);
      int32_t left_init = getLeftEncCount();
      int32_t right_init = getRightEncCount();
      LEDS_OFF;
      LED_On((SEARCH_SPEED / 100) % 4 + 1);

      while (!switches.key_pressed()) {

        if (getLeftEncCount() - left_init > 50) {
          SEARCH_SPEED += 100;
          left_init = getLeftEncCount();

          LEDS_OFF;
          LED_On((SEARCH_SPEED / 100) % 4 + 1);
        }

        else if (getLeftEncCount() - left_init < -50) {
          SEARCH_SPEED -= 100;
          left_init = getLeftEncCount();

          LEDS_OFF;
          LED_On((SEARCH_SPEED / 100) % 4 + 1);
        }

        if (getRightEncCount() - right_init > 60) {
          if (mouse.is_smooth_turn) {
            mouse.smooth_only = 1;
            LED_Blink(3, 100, 2);
          }
          mouse.is_smooth_turn = 1;
          right_init = getRightEncCount();

          LEDS_OFF;
          LED_Blink(2, 100, 2);
        }

        else if (getRightEncCount() - right_init < -50) {
          mouse.is_smooth_turn = 0;
          right_init = getRightEncCount();

          LEDS_OFF;
          LED_Blink(3, 100, 2);
        }

        if (switches.boot_pressed()) {
          mouse.trust_gyro = 1;
          LED1_ON;
          delay_ms(200);
        }

        if (SEARCH_SPEED < 100)
          SEARCH_SPEED = 100;
        if (SEARCH_SPEED > 1000)
          SEARCH_SPEED = 1000;

        delay_ms(10);
      }

      LEDS_OFF;
      while (switches.key_pressed())
        ;
      delay_ms(20);
    }

    static uint32_t lastTick = 0;
    if (millis() - lastTick >= 500) { // 500ms = 2 times per second
      LED2_TOGGLE;
      lowBatCheck();
      lastTick = millis();
    }

    if (is_run) {
      static uint8_t is_initialized = 0;
      if (!is_initialized) {
        icm_initialize();
        is_initialized = 1;
      }

      is_mouse_enable = 1;
      drive_enable();
      LED3_ON;
      set_steering_mode(STEER_NORMAL);
      motion.move(FULL_CELL * 8, 2000, 0, 4000);
      is_mouse_enable = 0;
      drive_disable();
      // print("Search started\n");
      // mouse.search(maze.goal());
      // maze.flood(START);
      is_run = 0;
    }

    if (is_calibrate) {
      cal_initial_wall();
      is_calibrate = 0;
    }

    if (is_icm_init) {
      icm_initialize();
      print("ICM initialized.\n");
      is_icm_init = 0;
    }

    if (is_collided) {
      safety_stop(50);
    }

    delay_ms(1);
  }
  return 0;
}
