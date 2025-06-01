#include "bmp280.h"


// Register map
#define BMP280_ID 0xD0
#define BMP280_ADDR 0x58


uint8_t bmp_read_reg(uint8_t reg) {
    uint8_t value = 0;
    HAL_Delay(30);
    if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY) != HAL_OK){
      Error_Handler();
    }
    HAL_Delay(30);
    return value;
}


HAL_StatusTypeDef bmp_heartbeat() {
    return bmp_read_reg(BMP280_ID) == BMP280_ADDR ? HAL_OK : HAL_ERROR;
}