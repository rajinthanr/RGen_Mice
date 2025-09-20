#include "wall_handle.h"
#include "core.h"
#include "math.h"
#include "delay.h"
#include "drive.h"


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

PID pid_theta = {30.0f, 0.3f, 0.1f, 0, 0};
PID pid_distance = {30.0f, 0.3f, 0.1f, 0, 0};

void Wall_Configuration(void)
{
    get_cal_initial_wall();
}

float dist(int ir_num)
{
    if (reading[ir_num] < 10)
        return 1000;

    else
    {
        float ratio = 1.0 * initial_wall[ir_num] / reading[ir_num];
        if (ratio >= 0)
        {
            return 5.0 * sqrt(ratio);
        }
        else
        {
            return 9000;
        }
    }
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
    delay_ms(2000);
    LED4_OFF;
    LED3_OFF;
    if(!pre_state) disable();
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
        printf("%d ", initial_wall[a]);
    }
    printf("        \n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool is_wall(int w)
{
    if (w == FL || w == FR)
        return (dis_reading[FL] + dis_reading[FR]) <= 20;
    return dis_reading[w] <= 10;
}

float clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    return value;
}

float wallFront(int band)
{
    float error_l = dis_reading[FL] - 5;
    float error_r = dis_reading[FR] - 5;

    // error_theta: difference between left and right front wall distances
    float error_theta = error_r - error_l;

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
    float correction_speed = 0;

    // error_distance: sum of left and right front wall errors (how far from target band)
    if (abs(error_theta) < 5)
    {
        float error_distance = error_l + error_r;

        pid_distance.integral += error_distance;
        float derivative_distance = error_distance - pid_distance.previous_error;
        pid_distance.previous_error = error_distance;
        pid_distance.integral = clamp(pid_distance.integral, -100, 100);

        correction_speed = pid_distance.kp * error_distance + pid_distance.ki * pid_distance.integral + pid_distance.kd * derivative_distance;
    }

    correction_angle = clamp(correction_angle, -400, 400);
    correction_speed = clamp(correction_speed, -400, 400);

    motion.set_target_omega(correction_angle );
    motion.set_target_velocity(correction_speed);

    return abs(correction_angle) + abs(correction_speed);
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
        error = dis_reading[R] - 5;
    }

    else if (include_left && is_wall(L) && !is_wall(R))
    {
        wall_state = 2;
        error = 5 - dis_reading[L];
    }
    else if (!is_wall(L) && !is_wall(R))
    {
        wall_state = 3;
        drive_dif(mouse.max_linear_speed, mouse.max_linear_speed);
    }

    pre_wall_state = wall_state;

    // Use error as input to wall theta PID using a new PID object
    static PID wall_theta_pid = {50.0f, 0.8f, 0.0f, 0, 0};

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
