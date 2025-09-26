#include "wall_handle.h"
#include "core.h"
#include "math.h"
#include "delay.h"
#include "drive.h"
#include "usart.h"


int wall_state, pre_wall_state;
bool is_calibrate = 0;
bool is_wall_front = 0;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
} PID;

PID pid_theta = {10.5f, 0.10f, 0.0f, 0, 0};
PID pid_distance = {30.0f, 0.3f, 0.1f, 0, 0};

void Wall_Configuration(void)
{
    get_cal_initial_wall();
}


void cal_initial_wall()
{
    uint8_t pre_state = is_sensor_active;
    enable();
    LED4_ON;
    delay_ms(1000);
    LED4_OFF;

    for (int a = 0; a < 4; a++)
    {
        initial_wall[a] = 0;
        wall_threshold[a] = 0;
    }

    for (int i = 0; i < 100; i++)
    {
        readSensor();
        for (int a = 0; a < 4; a++)
            initial_wall[a] += reading[a];
            delay_ms(1);
    }
    for (int a = 0; a < 4; a++)
    {
        initial_wall[a] = initial_wall[a] / 100;
        wall_threshold[a] = 250;
    }

    LED4_ON;
    LED3_ON;

    for (int a = 0; a < 4; a++)
        putInt(FLASH_CAL_INIT_WALL + a, initial_wall[a]);

    commit_flash();
    delay_ms(1000);
    LED4_OFF;
    LED3_OFF;
    if(!pre_state) disable();
    print("Calibration done\n");
}

void get_cal_initial_wall()
{
    //    initial_wall[0] = storage.getInt("ir_0");
    //    initial_wall[1] = storage.getInt("ir_1");
    //    initial_wall[2] = storage.getInt("ir_2");
    //    initial_wall[3] = storage.getInt("ir_3");
    for (int a = 0; a < 4; a++)
    {
        initial_wall[a] = getInt(FLASH_CAL_INIT_WALL + a);
        // wall_threshold[a] = 200;
        print("%d ", initial_wall[a]);
    }
    print("        \n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

float clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    return value;
}

float wallFront()
{
    float error_l = dis_reading[FL];
    float error_r = dis_reading[FR];

    // error_theta: difference between left and right front wall distances
    float error_theta = atan((error_r - error_l) / FRONT_SENSOR_SPACING)*180/PI;
    

    pid_theta.integral += error_theta;
    float derivative = error_theta - pid_theta.previous_error;
    pid_theta.previous_error = error_theta;

    pid_theta.integral = clamp(pid_theta.integral, -100, 100);
    if (reset_wall_pid)
    {
        pid_theta.integral = error_theta;
        derivative = 0;
        reset_wall_pid = false;
    }

    float correction_angle = pid_theta.kp * error_theta + pid_theta.ki * pid_theta.integral + pid_theta.kd * derivative;
  

    correction_angle = clamp(correction_angle, -STEERING_ADJUST_LIMIT*10, STEERING_ADJUST_LIMIT*10);

    mouse.front_adjustment = correction_angle;
    return abs(error_theta);
    //motion.set_target_velocity(correction_speed);

}

void wallFollow(bool include_left, bool include_right)
{
    float error = 0;

    if (include_left && is_wall(L) && include_right && is_wall(R))
    {
        wall_state = 0;

        error = (dis_reading[R] - dis_reading[L]) / 2;
    }

    else if (!is_wall(L) && include_right && is_wall(R))
    {
        // digitalWrite(LED_PIN, HIGH);
        wall_state = 1;
        error = dis_reading[R] - SIDE_WALL_DISTANCE_CAL;
    }

    else if (include_left && is_wall(L) && !is_wall(R))
    {
        wall_state = 2;
        error = SIDE_WALL_DISTANCE_CAL - dis_reading[L];
    }
    else if (!is_wall(L) && !is_wall(R))
    {
        wall_state = 3;
        return;
    }

    pre_wall_state = wall_state;

    // Use error as input to wall theta PID using a new PID object
    static PID wall_theta_pid = {5.0f, 0.08f, 0.01f, 0, 0};

    error = -error;

    wall_theta_pid.integral += error;
    float derivative = error - wall_theta_pid.previous_error;
    wall_theta_pid.previous_error = error;

    if (reset_wall_pid)
    {
        wall_theta_pid.integral = error;
        derivative = 0;
        reset_wall_pid = false;
    }

    wall_theta_pid.integral = clamp(wall_theta_pid.integral, -50, 50);

    float dif = wall_theta_pid.kp * error + wall_theta_pid.ki * wall_theta_pid.integral + wall_theta_pid.kd * derivative;

    dif = clamp(dif, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);

    mouse.steering_adjustment = dif;
}
