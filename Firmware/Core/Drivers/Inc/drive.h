#ifndef DRIVE_H
#define DRIVE_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t debug_mot; // Debug flag for motor outputs
extern uint8_t is_left_wall;
extern uint8_t is_right_wall;

// Legacy prototypes (not currently implemented in drive.cpp) -- consider
// removing or implementing
void drive_init(void); // TODO: implement or remove
void drive_set_speed(int left_speed, int right_speed); // TODO: map to drive_dif
void drive_stop(void); // TODO: implement stop logic

// Current drive interface implemented in drive.cpp
void drive(float speed,
           float angular_speed); // Linear (m/s) and angular (rad/s)
void drive_enable();
void reset_pwm();
void drive_init();
void drive_disable();

void drive_set_closed_loop(float linear, float angular);
void drive_closed_loop_update();

#ifdef __cplusplus
}
#endif

#endif // DRIVE_H
