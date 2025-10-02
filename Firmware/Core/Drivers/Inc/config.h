#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

//***************** STRUCTURES *****************//
struct TurnParameters {
  int speed;
  int entry_offset;
  int exit_offset;
  float angle;
  float omega;
  float alpha;
  int trigger;
};

//***************** CONSTANTS (DECLARATIONS) *****************//
extern const float PI;

extern const int FRONT_LEFT_CALIBRATION;
extern const int FRONT_RIGHT_CALIBRATION;
extern const int LEFT_CALIBRATION;
extern const int RIGHT_CALIBRATION;
extern const int FRONT_LINEAR_CONSTANT;
extern const int FRONT_REFERENCE;
extern const int TURN_THRESHOLD_SS90E;
extern const int EXTRA_WALL_ADJUST;

extern const float FULL_CELL;
extern const float HALF_CELL;
extern const float POLE_WIDTH;
extern const float SENSING_POSITION;

extern const int FRONT_SENSOR_SPACING;
extern const int SIDE_SENSOR_SPACING;
extern const int FRONT_SENSOR_DISPLACEMENT;
extern const int FRONT_WALL_DISTANCE_CAL;
extern const int SIDE_WALL_DISTANCE_CAL;

extern const int RFS_ADC_CHANNEL;
extern const int RSS_ADC_CHANNEL;
extern const int LSS_ADC_CHANNEL;
extern const int LFS_ADC_CHANNEL;

extern const uint32_t BAUDRATE;
extern const int REPORTING_INTERVAL;
extern const float BACK_WALL_TO_CENTER;

extern const float WHEEL_DIAMETER;
extern const float ENCODER_PULSES;
extern const float GEAR_RATIO;
extern const float MOUSE_RADIUS;
extern const float ROTATION_BIAS;

extern const float MM_PER_COUNT;
extern const float MM_PER_COUNT_LEFT;
extern const float MM_PER_COUNT_RIGHT;
extern const float DEG_PER_MM_DIFFERENCE;

extern const float LOOP_FREQUENCY;
extern const float LOOP_INTERVAL;

extern const float FWD_KM;
extern const float FWD_TM;
extern const float ROT_KM;
extern const float ROT_TM;

extern const float MAX_MOTOR_VOLTS;
extern const float SPEED_FF;
extern const float ACC_FF;
extern const float BIAS_FF;
extern const float TOP_SPEED;

extern const float FWD_ZETA;
extern const float FWD_TD;
extern const float FWD_KP;
extern const float FWD_KD;

extern const float ROT_ZETA;
extern const float ROT_TD;
extern const float ROT_KP;
extern const float ROT_KD;

extern const float STEERING_KP;
extern const float STEERING_KD;
extern const float STEERING_ADJUST_LIMIT;

#define ENCODER_LEFT_POLARITY (-1)
#define ENCODER_RIGHT_POLARITY (1)
#define MOTOR_LEFT_POLARITY (-1)
#define MOTOR_RIGHT_POLARITY (1)

extern int SEARCH_SPEED;
extern const int SEARCH_ACCELERATION;
extern const int SEARCH_TURN_SPEED;
extern const int SMOOTH_TURN_SPEED;
extern const int FAST_TURN_SPEED;
extern const int FAST_RUN_SPEED_MAX;
extern const float FAST_RUN_ACCELERATION;
extern const int OMEGA_SPIN_TURN;
extern const int ALPHA_SPIN_TURN;

extern const int SIDE_NOMINAL;
extern const int FRONT_NOMINAL;
extern const int FRONT_WALL_RELIABILITY_LIMIT;

extern const float FRONT_LEFT_SCALE;
extern const float FRONT_RIGHT_SCALE;
extern const float LEFT_SCALE;
extern const float RIGHT_SCALE;

extern const int LEFT_THRESHOLD;
extern const int RIGHT_THRESHOLD;
extern const int FRONT_THRESHOLD;

extern const int LEFT_EDGE_POS;
extern const int RIGHT_EDGE_POS;

extern const TurnParameters turn_params[4];

extern const float BATTERY_R1;
extern const float BATTERY_R2;
extern const float BATTERY_DIVIDER_RATIO;
extern const float ADC_FSR;
extern const float ADC_REF_VOLTS;
extern const float BATTERY_MULTIPLIER;

extern const int MOTOR_MAX_PWM;

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H
