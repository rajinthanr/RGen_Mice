#include "stm32f4xx.h"
#include "encoder.h"
#include "core.h"

// PA0	TIM5_CH1	Encoder_R_CHA
// PA1	TIM5_CH2	Encoder_R_CHB

// PA15	TIM2_CH1	Encoder_L_CHA
// PB3	TIM2_CH2	Encoder_L_CHB
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

#include <math.h>

#define ENCODER_TICKS_PER_100MM 220.0f
#define ENCODER_TICKS_PER_MM (ENCODER_TICKS_PER_100MM / 100.0f)
#define MM_PER_COUNT_LEFT (100.0f / ENCODER_TICKS_PER_100MM)
#define MM_PER_COUNT_RIGHT (100.0f / ENCODER_TICKS_PER_100MM)
#include "delay.h" // for micros()

volatile int32_t left_enc_last = 0;
volatile int32_t right_enc_last = 0;
static uint32_t left_enc_last_time = 0;
static uint32_t right_enc_last_time = 0;

float m_fwd_change = 0;
float m_rot_change = 0;
float m_robot_distance = 0;
float m_robot_angle = 0;

void update()
{
	int left_delta = 0;
	int right_delta = 0;
	left_delta = getLeftEncCount();
	right_delta = getRightEncCount();

	float left_change = left_delta * MM_PER_COUNT_LEFT;
	float right_change = right_delta * MM_PER_COUNT_RIGHT;
	m_fwd_change = 0.5 * (right_change + left_change);
	m_robot_distance += m_fwd_change;
	static float pre_angle = 0;
	m_rot_change = angle - pre_angle;
	m_robot_angle += m_rot_change;
	pre_angle = angle;
}

float robot_distance()
{
	return m_robot_distance;
}

float robot_angle()
{
	return m_robot_angle;
}

float robot_fwd_change()
{
	return m_fwd_change;
}

float robot_rot_change()
{
	return m_rot_change;
}

// Call this function periodically to get left wheel speed in mm/s
float getLeftSpeed(void)
{
	int32_t current = getLeftEncCount();
	uint32_t now = micros();
	float dt_sec = (now - left_enc_last_time) / 1000000.0f;
	int32_t ticks = current - left_enc_last;
	left_enc_last = current;
	left_enc_last_time = now;
	float distance_mm = ticks / ENCODER_TICKS_PER_MM;
	return (dt_sec > 0) ? (distance_mm / dt_sec) : 0.0f;
}

// Call this function periodically to get right wheel speed in mm/s
float getRightSpeed(void)
{
	int32_t current = getRightEncCount();
	uint32_t now = micros();
	float dt_sec = (now - right_enc_last_time) / 1000000.0f;
	int32_t ticks = current - right_enc_last;
	right_enc_last = current;
	right_enc_last_time = now;
	float distance_mm = ticks / ENCODER_TICKS_PER_MM;
	return (dt_sec > 0) ? (distance_mm / dt_sec) : 0.0f;
}

void Encoder_Configration(void)
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

int32_t getRightEncCount(void)
{
	uint16_t count = TIM2->CNT;

	static uint16_t pre_count = 0;
	static int16_t round = 0;

	if (abs(count - pre_count) > 50000)
	{
		if (count > 30000)
			round--;
		else
			round++;
	}

	pre_count = count;
	return -(round * 65536 + count);
}

void resetRightEncCount(void)
{
	TIM2->CNT = 0;
}

int32_t getLeftEncCount(void)
{
	uint16_t count = TIM1->CNT;

	static uint16_t pre_count = 0;
	static int16_t round = 0;

	if (abs(count - pre_count) > 50000)
	{
		if (count > 30000)
			round--;
		else
			round++;
	}

	pre_count = count;
	return (round * 65536 + count);
}

void resetLeftEncCount(void)
{
	TIM1->CNT = 0;
}
