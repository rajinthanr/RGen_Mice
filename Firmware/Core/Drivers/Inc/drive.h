#ifndef DRIVE_H
#define DRIVE_H

#include "stdbool.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    // Current state
    float linear_speed;  // Current linear speed (mm/s)
    float angular_speed; // Current angular speed (deg/s or rad/s)

    // Target state
    float target_linear_speed;  // Target linear speed (mm/s)
    float target_angular_speed; // Target angular speed (deg/s or rad/s)

    // Maximum limits
    float max_linear_speed;  // Maximum linear speed (mm/s)
    float max_angular_speed; // Maximum angular speed (deg/s or rad/s)

    // Acceleration limits
    float max_linear_accel;  // Maximum linear acceleration (mm/s^2)
    float max_angular_accel; // Maximum angular acceleration (deg/s^2 or rad/s^2)
} MouseMotionState;

    extern MouseMotionState mouse;

    // Legacy prototypes (not currently implemented in drive.cpp) -- consider removing or implementing
    void drive_init(void);                                 // TODO: implement or remove
    void drive_set_speed(int left_speed, int right_speed); // TODO: map to drive_dif
    void drive_stop(void);                                 // TODO: implement stop logic

    // Current drive interface implemented in drive.cpp
    void drive(float speed, float angular_speed); // Linear (m/s) and angular (rad/s)
    void drive_enable();                          // Enable/disable motor driver (PB2)
    void drive_disable();
    void drive_dif(float left_speed, float right_speed); // Differential speeds (-1..1)

    void drive_set_closed_loop(float linear, float angular);
    void drive_closed_loop_update();

#ifdef __cplusplus
}
#endif

#endif // DRIVE_H
