#include "stm32f4xx.h"
#include "encoder.h"

// PA0	TIM5_CH1	Encoder_R_CHA
// PA1	TIM5_CH2	Encoder_R_CHB

// PA15	TIM2_CH1	Encoder_L_CHA
// PB3	TIM2_CH2	Encoder_L_CHB
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

#include <math.h>

#define ENCODER_TICKS_PER_100MM 220.0f
#define ENCODER_TICKS_PER_MM (ENCODER_TICKS_PER_100MM / 100.0f)
#include "delay.h" // for micros()

volatile int32_t left_enc_last = 0;
volatile int32_t right_enc_last = 0;
static uint32_t left_enc_last_time = 0;
static uint32_t right_enc_last_time = 0;

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
