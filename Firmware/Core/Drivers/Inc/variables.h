#ifndef VARIABLES_H
#define VARIABLES_H

#ifdef __cplusplus
extern "C" {
#endif

#define COLUMNS 16 // x cells
#define ROWS 16    // y cells

#define FLASH_CAL_INIT_WALL 10
#define FLASH_K_WALL 20
#define FLASH_K_MOTOR 23
#define FLASH_K_FRONT 26

#include "stdint.h"
extern float CELL_DISTANCE; // in cm

extern bool debugSerial;

extern int flood[COLUMNS][ROWS];
extern bool ver_walls[COLUMNS + 1][ROWS];
extern bool hor_walls[COLUMNS][ROWS + 1];

extern int flood_fast[COLUMNS][ROWS];
extern bool ver_walls_fast[COLUMNS + 1][ROWS];
extern bool hor_walls_fast[COLUMNS][ROWS + 1];

extern int initial_wall[6];
extern int wall_threshold[4];

//+++++++++++++++++++++++++++++++++ common variables
//+++++++++++++++++++++++++++++++++++
extern bool btDebug;
extern bool btSerial;
extern bool fast_run_mode;

extern bool run_mode, settings_mode;

extern int phase;       // phases of arenas
extern bool next_phase; // to  increase phase after completing a cycle
extern float rover_speed, rover_acceleration, box_size;

extern bool enable_rover;
extern bool start_rover;

extern int ir_analog[8];

extern float dis; // temporarily store robots moved distance
extern float pre_dis;

extern unsigned long nowTime;

// OLED & BUTTONS
extern int selected;
extern int entered;

extern int current_arena;

//+++++++++++++++++++++++++++++++++ PID variables
//+++++++++++++++++++++++++++++++++++
extern long pTpid;
extern float kp;
extern float ki;
extern float kd;
extern float kpf, kif, kdf;
extern int pre_error;
extern int I;
extern float wall_f;

extern long pre_pid_time;
extern bool wall_follower, wall_front;
extern bool wall_processed;
extern bool flooded;
extern bool reset_wall_pid;

extern uint8_t x_pos;
extern uint8_t y_pos;
extern uint8_t end_x;
extern uint8_t end_y;
extern uint8_t head; // orientation

extern float start_dis;
extern bool travelled[COLUMNS][ROWS];

extern float kpgyro, kigyro, kdgyro;
extern float kpmotor, kimotor, kdmotor;
extern bool left_follow, right_follow;
extern bool total_angle;

#ifdef __cplusplus
}
#endif

#endif // VARIABLES_H