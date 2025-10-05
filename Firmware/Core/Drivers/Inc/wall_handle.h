#ifndef WALL_HANDLE_H
#define WALL_HANDLE_H

#ifdef __cplusplus
extern "C" {
#endif
#include "variables.h"

#define L 0
#define FL 1
#define FR 2
#define R 3

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
} PID;

extern bool is_calibrate;
extern bool is_wall_front;
extern PID wall_theta_pid;

void Wall_Configuration(void);
float sqrtf(float x);
void cal_initial_wall();
void get_cal_initial_wall();
float wallFront();
void wallFollow(bool include_left, bool include_right);

#ifdef __cplusplus
}
#endif

#endif // WALL_HANDLE_H