#ifndef WALL_HANDLE_H
#define WALL_HANDLE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "variables.h"

#define L 0
#define FL 1
#define FR 2
#define R 3

    extern bool is_calibrate;
    extern bool is_wall_front;

    void Wall_Configuration(void);
    float sqrtf(float x);
    float dist(int ir_num);
    void cal_initial_wall();
    void get_cal_initial_wall();
    float wallFront(int band);
    bool is_wall(int w);
    void wallFollow(bool include_left, bool include_right);

#ifdef __cplusplus
}
#endif

#endif // WALL_HANDLE_H