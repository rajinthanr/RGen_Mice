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

float acc_data_X;
float acc_data_Y;
float acc_data_Z;
float gyro_data_X;
float gyro_data_Y;
float gyro_data_Z;

float offset_acc_x = 0;
float offset_acc_y = 0;
float offset_acc_z = 0;
float offset_gyro_x = 0;
float offset_gyro_y = 0;
float offset_gyro_z = 0;

uint8_t fifo_data[16];

uint8_t acc_data_X1;
uint8_t acc_data_X0;

uint8_t acc_data_Y1;
uint8_t acc_data_Y0;

uint8_t acc_data_Z1;
uint8_t acc_data_Z0;

uint8_t gyro_data_X1;
uint8_t gyro_data_X0;

uint8_t gyro_data_Y1;
uint8_t gyro_data_Y0;

uint8_t gyro_data_Z1;
uint8_t gyro_data_Z0;

float acc_x_ms2;
float acc_y_ms2;
float acc_z_ms2;

float gyro_x_dps;
float gyro_y_dps;
float gyro_z_dps;

static void cs(int state)
{
	if (state)
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

void icm_initialize()
{
	// Set CS high initially
	cs(1);
	uint8_t configure_reset = 0x01;
	uint8_t fifo_conf_data = 0x03;
	uint8_t buffer = 0x0F; //  temperature sensor enabled. RC oscillator is on, gyro and accelerometer low noise mode,
	uint8_t init = 0x4E;
	uint8_t init_state = 0x0F;

	// Pull CS low to start SPI transaction
	cs(0);
	// HAL_SPI_Transmit(&hspi3, &configure_reset, 1, 100);
	HAL_Delay(100);
	// HAL_SPI_Transmit(&hspi3, &buffer, 1, 100);

	uint16_t accel_fs_sel = 0x40; // ACCEL_CONFIG: FS_SEL=2 (±8g), bits [3:2]=10
	uint16_t gyro_fs_sel = 0x00;  // GYRO_CONFIG: FS_SEL=0 (±2000 dps), bits [4:3]=00	

	// Write ACCEL_CONFIG register (address 0x14)
	uint16_t accel_config_reg = 0x50;
	cs(0);
	HAL_SPI_Transmit(&hspi3, &accel_config_reg, 1, 100);
	HAL_SPI_Transmit(&hspi3, &accel_fs_sel, 1, 100);
	cs(1);
	HAL_Delay(100);

	// Write GYRO_CONFIG register (address 0x11)
	uint16_t gyro_config_reg = 0x4F;
	cs(0);
	HAL_SPI_Transmit(&hspi3, &gyro_config_reg, 1, 100);
	HAL_SPI_Transmit(&hspi3, &gyro_fs_sel, 1, 100);
	cs(1);
	HAL_Delay(100);
	cs(0);
	HAL_SPI_Transmit(&hspi3, &init, 1, 100);
	HAL_Delay(100);
	HAL_SPI_Transmit(&hspi3, &init_state, 1, 100);
	cs(1);
	// HAL_SPI_Transmit(&hspi3, &fifo_conf_data, 1, 100);
	HAL_Delay(100);
	// Pull CS high to end SPI transaction

	uint8_t whoami_reg = 0x75 | 0x80; // WHO_AM_I register address with read bit set
	uint8_t whoami_val = 0;
	cs(0);
	HAL_SPI_Transmit(&hspi3, &whoami_reg, 1, 100);
	HAL_SPI_Receive(&hspi3, &whoami_val, 1, 100);
	// Pull CS high to end SPI transaction
	cs(1);

	float avg_acc_x = 0;
	float avg_acc_y = 0;
	float avg_acc_z = 0;
	float avg_gyro_x = 0;
	float avg_gyro_y = 0;
	float avg_gyro_z = 0;

	for (int i = 0; i < 1000; i++)
	{
		read_values();
		avg_acc_x += acc_x_ms2;
		avg_acc_y += acc_y_ms2;
		avg_acc_z += acc_z_ms2;
		avg_gyro_x += gyro_data_X;
		avg_gyro_y += gyro_data_Y;
		avg_gyro_z += gyro_data_Z;
	}
	avg_acc_x /= 1000;
	avg_acc_y /= 1000;
	avg_acc_z /= 1000;
	avg_gyro_x /= 1000;
	avg_gyro_y /= 1000;
	avg_gyro_z /= 1000;

	offset_acc_x = avg_acc_x;
	offset_acc_y = avg_acc_y;
	offset_acc_z = avg_acc_z;
	offset_gyro_x = avg_gyro_x;
	offset_gyro_y = avg_gyro_y;
	offset_gyro_z = avg_gyro_z;
}

void read_values()
{
	// Pull CS low to start SPI transaction
	cs(0);
	uint8_t reg_addr = 0x1F | 0x80; // Register address with read bit set
	HAL_SPI_Transmit(&hspi3, &reg_addr, 1, 100);
	HAL_SPI_Receive(&hspi3, fifo_data, 16, 100);
	// Pull CS high to end SPI transaction
	cs(1);

	acc_data_X1 = fifo_data[0];
	acc_data_X0 = fifo_data[1];
	acc_data_X = (acc_data_X1 << 8) | acc_data_X0;

	acc_data_Y1 = fifo_data[2];
	acc_data_Y0 = fifo_data[3];
	acc_data_Y = (acc_data_Y1 << 8) | acc_data_Y0;

	acc_data_Z1 = fifo_data[4];
	acc_data_Z0 = fifo_data[5];
	acc_data_Z = (acc_data_Z1 << 8) | acc_data_Z0;

	gyro_data_X1 = fifo_data[6];
	gyro_data_X0 = fifo_data[7];
	gyro_data_X = (gyro_data_X1 << 8) | gyro_data_X0;

	gyro_data_Y1 = fifo_data[8];
	gyro_data_Y0 = fifo_data[9];
	gyro_data_Y = (gyro_data_Y1 << 8) | gyro_data_Y0;

	gyro_data_Z1 = fifo_data[10];
	gyro_data_Z0 = fifo_data[11];
	gyro_data_Z = (gyro_data_Z1 << 8) | gyro_data_Z0;

	// Accelerometer: convert raw values to m/s^2 (assuming ±4g, 16-bit, 1g = 9.80665 m/s^2)
	// Sensitivity for ±4g: 8192 LSB/g
	acc_x_ms2 = ((int16_t)acc_data_X) * 9.80665f / 8192.0f;
	acc_y_ms2 = ((int16_t)acc_data_Y) * 9.80665f / 8192.0f;
	acc_z_ms2 = ((int16_t)acc_data_Z) * 9.80665f / 8192.0f;

	// Gyroscope: convert raw values to dps (assuming ±500 dps, 16-bit)
	// Sensitivity for ±500 dps: 65.5 LSB/(°/s)
	gyro_x_dps = ((int16_t)gyro_data_X) / 65.5f;
	gyro_y_dps = ((int16_t)gyro_data_Y) / 65.5f;
	gyro_z_dps = ((int16_t)gyro_data_Z) / 65.5f;
}

float get_accY(){
	uint8_t data[2];
	cs(0);
	uint8_t reg_addr = 0x21 | 0x80; // Register address with read bit set
	HAL_SPI_Transmit(&hspi3, &reg_addr, 1, 100);
	HAL_SPI_Receive(&hspi3, data, 2, 100);
	// Pull CS high to end SPI transaction
	cs(1);

	acc_data_Y0 = data[0];
	acc_data_Y1 = data[1];
	acc_data_Y = (acc_data_Y1 << 8) | acc_data_Y0;

	float val = ((int16_t)acc_data_Y) * 9.80665f / 8192.0f - offset_acc_y;
	return val;
}

float get_gyroZ()
{
	// Pull CS low to start SPI transaction
	uint8_t data[2];
	cs(0);
	uint8_t reg_addr = 0x29 | 0x80; // Register address with read bit set
	HAL_SPI_Transmit(&hspi3, &reg_addr, 1, 100);
	HAL_SPI_Receive(&hspi3, data, 2, 100);
	// Pull CS high to end SPI transaction
	cs(1);

	gyro_data_Z1 = data[0];
	gyro_data_Z0 = data[1];
	gyro_data_Z = ((gyro_data_Z1 << 8) | gyro_data_Z0) - offset_gyro_z;

	float gyro_z_dps = ((int16_t)gyro_data_Z) / 16.4f;
	return (gyro_z_dps)*3555/3600;
}
