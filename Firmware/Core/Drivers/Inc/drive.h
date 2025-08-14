#ifndef DRIVE_H
#define DRIVE_H

#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

// Legacy prototypes (not currently implemented in drive.cpp) -- consider removing or implementing
void drive_init(void);          // TODO: implement or remove
void drive_set_speed(int left_speed, int right_speed); // TODO: map to drive_dif
void drive_stop(void);          // TODO: implement stop logic

// Current drive interface implemented in drive.cpp
void drive(float speed, float angular_speed);          // Linear (m/s) and angular (rad/s)
void drive_enable();                        // Enable/disable motor driver (PB2)
void drive_disable();
void drive_dif(float left_speed, float right_speed);   // Differential speeds (-1..1)

#ifdef __cplusplus
}
#endif

#endif // DRIVE_H