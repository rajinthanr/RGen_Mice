#define COLUMNS 16 // x cells
#define ROWS 16    // y cells

#include "stdint.h"
float CELL_DISTANCE = 18; // in cm

int flood[COLUMNS][ROWS];
bool ver_walls[COLUMNS + 1][ROWS];
bool hor_walls[COLUMNS][ROWS + 1];

int flood_fast[COLUMNS][ROWS];
bool ver_walls_fast[COLUMNS + 1][ROWS];
bool hor_walls_fast[COLUMNS][ROWS + 1];

int initial_wall[6] = {};
int wall_threshold[4] = {};

//+++++++++++++++++++++++++++++++++ common variables +++++++++++++++++++++++++++++++++++
bool btDebug = 1;
bool btSerial;

bool run_mode = 0, settings_mode = 0;

int phase = 0;   // phases of arenas
bool next_phase; // to  increase phase after compleating a cycle
float rover_speed, rover_acceleration, box_size;

bool enable_rover = 0;
bool start_rover = 0;

int ir_analog[8];

float dis = 0; // temporarly store robots moved distance
float pre_dis = 0;

bool fast_run_mode = 0;

unsigned long nowTime;

// OLED & BUTTONS
int selected = 0;
int entered = -1;

int current_arena = 0;

//+++++++++++++++++++++++++++++++++ PID variables +++++++++++++++++++++++++++++++++++
long pTpid;
float kp = 0.1;
float ki = 0;
float kd = 0;
float kpf = 0, kif = 0, kdf = 0;

float wall_f;
bool debugSerial;

bool wall_follower, wall_front;
bool wall_processed = false;
bool flooded;
bool reset_wall_pid = false;

uint8_t x_pos = 0;
uint8_t y_pos = 0;
uint8_t end_x = COLUMNS / 2 - 1;
uint8_t end_y = ROWS / 2 - 1;
uint8_t head = 0; // orientation

float start_dis = 0;
bool travelled[COLUMNS][ROWS];

float kpgyro, kigyro, kdgyro;
float kpmotor, kimotor, kdmotor;
bool left_follow, right_follow;
uint32_t total_angle;