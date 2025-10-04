#include "flash.h"
#include "core.h"
#include "main.h"

#define FLASHWORD 1
#define NUM_WORDS 1024
uint32_t flash_data[1024];

void init_flash() { read_flash(); }

void commit_flash() {
  static uint32_t last_write_time = -2000;
  uint32_t current_time =
      HAL_GetTick(); // Assuming HAL_GetTick() returns time in milliseconds

  // Prevent writing if the last write was less than 1000 milliseconds ago
  if (current_time - last_write_time < 1000) {
    return;
  }
  Flash_Write_Data(flash_data, NUM_WORDS);
  last_write_time = current_time;
}

void read_flash() { Flash_Read_Data(flash_data, NUM_WORDS); }

void clear_flash() {
  for (int i = 0; i < NUM_WORDS; i++) {
    flash_data[i] = 0;
  }
  commit_flash();
}

void putInt(int index, int value) { flash_data[index] = *(int *)&value; }

int getInt(int index) { return *(int *)&flash_data[index]; }

void putFloat(int index, float value) {
  flash_data[index] = *(uint32_t *)&value;
}

float getFloat(int index) { return *(float *)&flash_data[index]; }

uint32_t Flash_Write_Data(uint32_t *data, uint16_t numberofwords = NUM_WORDS) {
  uint32_t StartSectorAddress = 0x08060000;

  static FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SECTORError;
  int sofar = 0;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area */
  /* Get the number of sector to erase from 1st sector */

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = 7; // the last sector

  // The the proper BANK to erase the Sector
  EraseInitStruct.Banks = FLASH_BANK_1;
  EraseInitStruct.NbSectors = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
    return HAL_FLASH_GetError();

  /* Program the user Flash area 1 WORDS at a time
   * (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR)
   * ***********/

  while (sofar < numberofwords) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress,
                          (uint32_t)data[sofar]) == HAL_OK) {
      StartSectorAddress += 4 * FLASHWORD; // increment the address by 4 bytes
      sofar += FLASHWORD;
    } else {
      /* Error occurred while writing data in Flash memory*/
      return HAL_FLASH_GetError();
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return 0;
}

void Flash_Read_Data(uint32_t *data, uint16_t numberofwords = NUM_WORDS) {
  uint32_t StartSectorAddress = 0x08060000;
  while (1) {
    *data = *(__IO uint32_t *)StartSectorAddress;
    StartSectorAddress += 4;
    data++;
    if (!(numberofwords--))
      break;
  }
}
