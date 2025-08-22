//#include "global.h"
#include "main.h"
#include "sensor_Function.h"
#include "adc.h"
#include "delay.h"
#include "pwm.h"
#include "led.h"
#include <stdio.h>
#include "encoder.h"

int reflectionRate = 1000;//which is 1.000 (converted to ingeter)

int32_t volMeter = 0;
int32_t voltage = 0;
int32_t LSensor = 0;
int32_t RSensor = 0;
int32_t FLSensor = 0;
int32_t FRSensor = 0;
int32_t Outz = 0;
int32_t aSpeed = 0;//angular velocity
int32_t angle = 0; 


/*read IR sensors*/
void readSensor(void)
{
	uint32_t curt;
//read DC value		
	
	LSensor = read_L_Sensor;
	RSensor = read_R_Sensor;
	FLSensor = read_FL_Sensor;
	FRSensor = read_FR_Sensor;
	
	curt = micros();
	
//left front sensor
	L_EM_ON;
	elapseMicros(60,curt);
	LSensor = read_L_Sensor - LSensor;
	L_EM_OFF;
	if(LSensor < 0)//error check
		LSensor = 0;
 	elapseMicros(140,curt);
//right front sensor	
	R_EM_ON;
	elapseMicros(200,curt);	
	RSensor = read_R_Sensor - RSensor;
	R_EM_OFF;
	if(RSensor < 0)
		RSensor = 0;
 	elapseMicros(280,curt);
//diagonal sensors
 	FL_EM_ON;FR_EM_ON;
	elapseMicros(340,curt);	
	FLSensor = read_FL_Sensor - FLSensor;
	FRSensor = read_FR_Sensor - FRSensor;
    FL_EM_OFF;FR_EM_OFF;
	if(FLSensor < 0)
		FLSensor = 0;
	if(FRSensor < 0)
		FRSensor = 0;
	
	readVolMeter();
	
	LSensor = LSensor*reflectionRate/1000;
	RSensor = RSensor*reflectionRate/1000;
	FLSensor = FLSensor*reflectionRate/1000;
	FRSensor = FRSensor*reflectionRate/1000;
	
	//delay_us(80);
	//elapseMicros(500,curt);
}
//there are 1000 - 340 = 660 us remaining in a 1ms_ISR

/*read gyro*/
void readGyro(void)
{	                      //k=19791(sum for sample in 1 second)    101376287 for 50 seconds with 5000 samples
	int i;
	int sampleNum = 20;
	aSpeed = 0;
	for(i=0;i<sampleNum;i++)
		//aSpeed += read_Outz;
    aSpeed *= 50000/sampleNum;
	aSpeed -= 92980000;
	aSpeed /= 50000;	
	aSpeed /= 4;
	angle += aSpeed; 
}


/*read voltage meter*/
void readVolMeter(void)
{          //3240 = 7.85V
	volMeter = read_Vol_Meter;//raw value
	voltage = volMeter*809/3248;//actual voltage value  ex) 8.2V = 8200
}

void lowBatCheck(void)
{
  if(voltage < 700) //alert when battery Voltage lower than 7V
	{	
		
		setLeftPwm(0);
		setRightPwm(0);
		
		while(1)
		{
			ALL_LED_OFF;
			delay_ms(200);

			ALL_LED_ON;
			delay_ms(200);			
		}
	}
  else {
		  delay_ms(1000);
	}		
}


void IR_Configuration(void) {
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

