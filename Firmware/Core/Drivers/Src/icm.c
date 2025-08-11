// Helper function to control the CS pin
/*
 * icm.c
 *
 *  Created on: Dec 22, 2020
 *      Author: Mert Kilic
 */
#include "icm.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

uint8_t fifo_data[16];

uint8_t  acc_data_X1;
uint8_t  acc_data_X0;
extern	uint16_t acc_data_X;

uint8_t  acc_data_Y1;
uint8_t  acc_data_Y0;
extern	uint16_t acc_data_Y;

uint8_t  acc_data_Z1;
uint8_t  acc_data_Z0;
extern	uint16_t acc_data_Z;

uint8_t  gyro_data_X1;
uint8_t  gyro_data_X0;
extern	uint16_t gyro_data_X;

uint8_t  gyro_data_Y1;
uint8_t  gyro_data_Y0;
extern	uint16_t gyro_data_Y;

uint8_t  gyro_data_Z1;
uint8_t  gyro_data_Z0;
extern	uint16_t gyro_data_Z;


static void cs(int state) {
	if (state)
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}



void icm_initialize(){
	// Set CS high initially
	cs(1);
	uint8_t configure_reset = 0x01;
	uint8_t fifo_conf_data = 0x03;
	uint8_t buffer = 0x0F; //  temperature sensor enabled. RC oscillator is on, gyro and accelerometer low noise mode,
	uint8_t init = 0x4E;
    uint8_t init_state = 0x0F;

	// Pull CS low to start SPI transaction
	cs(0);
	//HAL_SPI_Transmit(&hspi3, &configure_reset, 1, 100);
	HAL_Delay(100);
    //HAL_SPI_Transmit(&hspi3, &buffer, 1, 100);
	HAL_Delay(100);
	HAL_SPI_Transmit(&hspi3, &init, 1, 100);
    HAL_SPI_Transmit(&hspi3, &init_state, 1, 100);
	//HAL_SPI_Transmit(&hspi3, &fifo_conf_data, 1, 100);
	HAL_Delay(100);
	// Pull CS high to end SPI transaction
	cs(1);
    
    uint8_t whoami_reg = 0x75 | 0x80; // WHO_AM_I register address with read bit set
    uint8_t whoami_val = 0;
    cs(0);
    HAL_SPI_Transmit(&hspi3, &whoami_reg, 1, 100);
    HAL_SPI_Receive(&hspi3, &whoami_val, 1, 100);
    // Pull CS high to end SPI transaction
    cs(1);
}


void read_values(){
	// Pull CS low to start SPI transaction
	cs(0);
    uint8_t reg_addr = 0x1F | 0x80; // Register address with read bit set
    HAL_SPI_Transmit(&hspi3, &reg_addr, 1, 100);
	HAL_SPI_Receive(&hspi3, fifo_data, 16, 100);
	// Pull CS high to end SPI transaction
	cs(1);

	  acc_data_X1 = fifo_data[0];
	  acc_data_X0 = fifo_data[1];
	  acc_data_X = (acc_data_X1<<8) | acc_data_X0;

	  acc_data_Y1 = fifo_data[2];
	  acc_data_Y0 = fifo_data[3];
	  acc_data_Y = (acc_data_Y1<<8) | acc_data_Y0;

	  acc_data_Z1 = fifo_data[4];
	  acc_data_Z0 = fifo_data[5];
	  acc_data_Z = (acc_data_Z1<<8) | acc_data_Z0;

	  gyro_data_X1 = fifo_data[6];
	  gyro_data_X0 = fifo_data[7];
	  gyro_data_X = (gyro_data_X1<<8) | gyro_data_X0;

	  gyro_data_Y1 = fifo_data[8];
	  gyro_data_Y0 = fifo_data[9];
	  gyro_data_Y = (gyro_data_Y1<<8) | gyro_data_Y0;

	  gyro_data_Z1 = fifo_data[10];
	  gyro_data_Z0 = fifo_data[11];
	  gyro_data_Z = (gyro_data_Z1<<8) | gyro_data_Z0;


}