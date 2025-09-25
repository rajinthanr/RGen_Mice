// #include "global.h"
#include "main.h"
#include "sensor_Function.h"
#include "adc.h"
#include "delay.h"
#include "pwm.h"
#include "led.h"
#include "encoder.h"
#include "icm.h"
#include "core.h"

int reflectionRate = 1000; // which is 1.000 (converted to ingeter)
float cell_1 = 0;
float cell_2 = 0;

int32_t volMeter = 0;
int32_t voltage = 0;
int32_t LSensor = 0;
int32_t RSensor = 0;
int32_t FLSensor = 0;
int32_t FRSensor = 0;
int32_t Outz = 0;
float aSpeed = 0; // angular velocity
float angle = 0;
int reading[4];
float dis_reading[4];
uint8_t is_sensor_active = false;

uint8_t NO_START = 0;
uint8_t LEFT_START = 1;
uint8_t RIGHT_START = 2;


float dist(int ir_num)
{
    if (reading[ir_num] < 10)
        return 1000;

    else
    {
        float ratio = 1.0 * initial_wall[ir_num] / reading[ir_num];
        if (ratio >= 0)
        {
			float distance = 0;
			if(ir_num == L || ir_num == R) distance = SIDE_WALL_DISTANCE_CAL*sqrt(ratio);
			else distance = FRONT_WALL_DISTANCE_CAL*sqrt(ratio);

         return distance;
        }
        else
        {
            return 9000;
        }
    }
}


bool is_wall(int w)
{
    if (w == FL || w == FR)
        return (dis_reading[FL] + dis_reading[FR]) <= 400;
    return dis_reading[w] <= 120;
}

void set_steering_mode(uint8_t mode) {
	if(mouse.steering_mode == GYRO_OFF){
		angle = mouse.target_angle;
	}
   mouse.last_steering_error = 0;
   mouse.steering_adjustment = 0;
   mouse.steering_mode = mode;
  }

/*read IR sensors*/
void readSensor(void)
{
	if(is_sensor_active == false) return;
	uint32_t curt;
	// read DC value

	LSensor = read_L_Sensor;
	RSensor = read_R_Sensor;
	FLSensor = read_FL_Sensor;
	FRSensor = read_FR_Sensor;

	curt = micros();

	// left front sensor
	L_EM_ON;
	elapseMicros(80, curt);
	LSensor = read_L_Sensor - LSensor;
	L_EM_OFF;
	if (LSensor < 0) // error check
		LSensor = 0;
	//elapseMicros(140, curt);
	// right front sensor
	R_EM_ON;
	elapseMicros(160, curt);
	RSensor = read_R_Sensor - RSensor;
	R_EM_OFF;
	if (RSensor < 0)
		RSensor = 0;
	//elapseMicros(280, curt);
	// diagonal sensors
	FL_EM_ON;
	FR_EM_ON;
	elapseMicros(240, curt);
	FLSensor = read_FL_Sensor - FLSensor;
	FRSensor = read_FR_Sensor - FRSensor;
	FL_EM_OFF;
	FR_EM_OFF;
	if (FLSensor < 0)
		FLSensor = 0;
	if (FRSensor < 0)
		FRSensor = 0;

	//readVolMeter();

	LSensor = LSensor * reflectionRate / 1000;
	RSensor = RSensor * reflectionRate / 1000;
	FLSensor = FLSensor * reflectionRate / 1000;
	FRSensor = FRSensor * reflectionRate / 1000;

	reading[0] = LSensor;
	reading[1] = FLSensor;
	reading[2] = FRSensor;
	reading[3] = RSensor;

	for (int i = 0; i < 4; i++)
	{
		dis_reading[i] = dist(i);
	}

	// delay_us(80);
	// elapseMicros(500,curt);
}
// there are 1000 - 340 = 660 us remaining in a 1ms_ISR

float get_front_sum()
{
	return dis_reading[FL] + dis_reading[FR];
}

/*read gyro*/
void readGyro(void)
{ // k=19791(sum for sample in 1 second)    101376287 for 50 seconds with 5000 samples
	aSpeed = get_gyroZ();

	float LOOP_INTERVAL = 0.001;
	angle += aSpeed * LOOP_INTERVAL;
}

void safety_stop(int duration = 100)
{
	drive_disable();
	drive(0, 0);
	ALL_LED_OFF;

	while (1)
	{
		ALL_LED_OFF;
		delay_ms(duration*4);

		LED3_ON;
		delay_ms(duration);
	}
}

void collisionAvoidance(void)
{
	if (abs(get_accY()) > 40) // If acceleration in Y direction exceeds 2 m/s^2
	{
		print("Collision detected!");
		safety_stop(100);
	}
}

/*read voltage meter*/
void readVolMeter(void)
{										// 3240 = 7.85V
	volMeter = read_Vol_Meter; // raw value
	float c3_7 = readADC(2);
	float v7_4 = readADC(3);

	c3_7 = c3_7 * 0.488 * 3.3;
	v7_4 = v7_4 * 0.731 * 3.3;

	cell_1 = c3_7;
	cell_2 = v7_4 - c3_7;
}

void lowBatCheck(void)
{
	if (cell_1 < 3500 || cell_2 < 3500) // alert when battery Voltage lower than 7V
	{
		print("Low battery detected!");
		safety_stop();
	}
}

void IR_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = TR_L_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TR_L_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TR_R_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TR_R_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TR_FL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TR_FL_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TR_FR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TR_FR_GPIO_Port, &GPIO_InitStruct);
}

void enable(){
	is_sensor_active = true;
}

void disable(){
	is_sensor_active = false;
}

uint8_t occluded_left() {
    return dis_reading[FL] < 50 && dis_reading[FR] > 50 ;
  }

uint8_t occluded_right() {
    return dis_reading[FL] > 50 && dis_reading[FR] < 50;
  }


uint8_t wait_for_user_start() {
    int state = 0;
    LED3_ON;
    enable();
    uint8_t choice = NO_START;
    while (choice == NO_START) {
      int count = 0;
      while (occluded_left()) {
        count++;
        delay_ms(20);
      }
      if (count > 5) {
        choice = LEFT_START;
        break;
      }
      count = 0;
      while (occluded_right()) {
        count++;
        delay_ms(20);
      }
      if (count > 5) {
        choice = RIGHT_START;
        break;
      }
      LED3_ON;
      state = 1 - state;
      delay_ms(25);
    }
    disable();
    LED3_OFF;
    delay_ms(250);
    return choice;
  }
