#include "config.h"

//***************** DEFINITIONS *****************//

const float PI = 3.141592f;

const int FRONT_LEFT_CALIBRATION = 85;
const int FRONT_RIGHT_CALIBRATION = 90;
const int LEFT_CALIBRATION = 110;
const int RIGHT_CALIBRATION = 80;
const int FRONT_LINEAR_CONSTANT = 1030;
const int FRONT_REFERENCE = 850;
const int TURN_THRESHOLD_SS90E = 82;
const int EXTRA_WALL_ADJUST = 5;

const float FULL_CELL = 192.0f;
const float HALF_CELL = FULL_CELL / 2.0f;
const float POLE_WIDTH = 12.0f;
const float SENSING_POSITION = 190.0f;

const int FRONT_SENSOR_SPACING = 50;
const int SIDE_SENSOR_SPACING = 24;
const int FRONT_SENSOR_DISPLACEMENT = 23;
const int FRONT_WALL_DISTANCE_CAL = 100;
const int SIDE_WALL_DISTANCE_CAL =
    (FULL_CELL - POLE_WIDTH - SIDE_SENSOR_SPACING) / 2;

const int RFS_ADC_CHANNEL = 0;
const int RSS_ADC_CHANNEL = 1;
const int LSS_ADC_CHANNEL = 2;
const int LFS_ADC_CHANNEL = 3;

const uint32_t BAUDRATE = 115200;
const int REPORTING_INTERVAL = 10;
const float BACK_WALL_TO_CENTER = 45.0f;

const float WHEEL_DIAMETER = 18;
const float ENCODER_PULSES = 40;
const float GEAR_RATIO = 6;
const float MOUSE_RADIUS = 55.0f / 2.0f;
const float ROTATION_BIAS = 0.0025;

const float MM_PER_COUNT = PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * MM_PER_COUNT;
const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * MM_PER_COUNT;
const float DEG_PER_MM_DIFFERENCE = (180.0f / (2 * MOUSE_RADIUS * PI));

const float LOOP_FREQUENCY = 1000.0f;
const float LOOP_INTERVAL = (1.0f / LOOP_FREQUENCY);

const float FWD_KM = 475.0f;
const float FWD_TM = 0.190f;
const float ROT_KM = 775.0f;
const float ROT_TM = 0.210f;

const float MAX_MOTOR_VOLTS = 6.0f;
const float SPEED_FF = (1.0f / FWD_KM);
const float ACC_FF = (FWD_TM / FWD_KM);
const float BIAS_FF = 0.121f;
const float TOP_SPEED = (6.0f - BIAS_FF) / SPEED_FF;

const float FWD_ZETA = 0.707f;
const float FWD_TD = FWD_TM;
const float FWD_KP =
    16 * FWD_TM / (FWD_KM * FWD_ZETA * FWD_ZETA * FWD_TD * FWD_TD);
const float FWD_KD = LOOP_FREQUENCY * (8 * FWD_TM - FWD_TD) / (FWD_KM * FWD_TD);

const float ROT_ZETA = 0.707f;
const float ROT_TD = ROT_TM;
const float ROT_KP =
    16 * ROT_TM / (ROT_KM * ROT_ZETA * ROT_ZETA * ROT_TD * ROT_TD);
const float ROT_KD = LOOP_FREQUENCY * (8 * ROT_TM - ROT_TD) / (ROT_KM * ROT_TD);

const float STEERING_KP = 0.25f;
const float STEERING_KD = 0.0f;
const float STEERING_ADJUST_LIMIT = 10.0f;

int SEARCH_SPEED = 200;
const int SEARCH_ACCELERATION = 2000;
const int SEARCH_TURN_SPEED = 100;
const int SMOOTH_TURN_SPEED = 500;
const int FAST_TURN_SPEED = 600;
const int FAST_RUN_SPEED_MAX = 2500;
const float FAST_RUN_ACCELERATION = 3000;
const int OMEGA_SPIN_TURN = 360;
const int ALPHA_SPIN_TURN = 3600;

const int SIDE_NOMINAL = 100;
const int FRONT_NOMINAL = 100;
const int FRONT_WALL_RELIABILITY_LIMIT = 100;

const float FRONT_LEFT_SCALE = (float)SIDE_NOMINAL / FRONT_LEFT_CALIBRATION;
const float FRONT_RIGHT_SCALE = (float)FRONT_NOMINAL / FRONT_RIGHT_CALIBRATION;
const float LEFT_SCALE = (float)SIDE_NOMINAL / LEFT_CALIBRATION;
const float RIGHT_SCALE = (float)SIDE_NOMINAL / RIGHT_CALIBRATION;

const int LEFT_THRESHOLD = 40;
const int RIGHT_THRESHOLD = 40;
const int FRONT_THRESHOLD = 20;

const int LEFT_EDGE_POS = 90;
const int RIGHT_EDGE_POS = 90;

const TurnParameters turn_params[4] = {
    {SEARCH_TURN_SPEED, 70, 80, 90.0f, 287.0f, 2866.0f, TURN_THRESHOLD_SS90E},
    {SEARCH_TURN_SPEED, 70, 80, -90.0f, 287.0f, 2866.0f, TURN_THRESHOLD_SS90E},
    {SEARCH_TURN_SPEED, 70, 80, 90.0f, 287.0f, 2866.0f, TURN_THRESHOLD_SS90E},
    {SEARCH_TURN_SPEED, 70, 80, -90.0f, 287.0f, 2866.0f, TURN_THRESHOLD_SS90E},
};

const float BATTERY_R1 = 10000.0f;
const float BATTERY_R2 = 10000.0f;
const float BATTERY_DIVIDER_RATIO = BATTERY_R2 / (BATTERY_R1 + BATTERY_R2);
const float ADC_FSR = 1023.0f;
const float ADC_REF_VOLTS = 5.0f;
const float BATTERY_MULTIPLIER =
    (ADC_REF_VOLTS / ADC_FSR / BATTERY_DIVIDER_RATIO);

const int MOTOR_MAX_PWM = 255;
