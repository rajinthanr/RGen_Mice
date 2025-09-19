#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stm32f4xx_hal.h"

    extern uint32_t flash_data[1024];
    void init_flash();
    void commit_flash();
    void read_flash();
    void clear_flash();
    void putInt(int index, int value);
    int getInt(int index);
    void putFloat(int index, float value);
    float getFloat(int index);

    uint32_t Flash_Write_Data(uint32_t *data, uint16_t numberofwords);
    void Flash_Read_Data(uint32_t *data, uint16_t numberofwords);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_SECTOR_H7_H_ */
