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

// Current drive interface implemented in drive.cpp
void drive(float left_pwm, float right_pwm);
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
