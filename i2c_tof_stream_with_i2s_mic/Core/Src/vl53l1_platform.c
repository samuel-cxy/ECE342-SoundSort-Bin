#include "vl53l1_platform.h"
#include "main.h"

// Tell the file to use your I2C1 handle from main.c
extern I2C_HandleTypeDef hi2c1;

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    HAL_I2C_Mem_Write(&hi2c1, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, count, 100);
    return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    HAL_I2C_Mem_Read(&hi2c1, dev, index, I2C_MEMADD_SIZE_16BIT, pdata, count, 100);
    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buf[2] = {(data >> 8) & 0xFF, data & 0xFF};
    return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buf[4] = {(data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
    return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata) {
    return VL53L1_ReadMulti(dev, index, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata) {
    uint8_t buf[2];
    VL53L1_ReadMulti(dev, index, buf, 2);
    *pdata = (buf[0] << 8) | buf[1];
    return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata) {
    uint8_t buf[4];
    VL53L1_ReadMulti(dev, index, buf, 4);
    *pdata = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    HAL_Delay(wait_ms);
    return 0;
}
