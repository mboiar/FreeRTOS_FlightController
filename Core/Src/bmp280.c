#include "bmp280.h"
#include "i2c.h"

// Register map
#define BMP280_ID 0xD0
#define BMP280_ID_VALUE 0x58
#define BMP280_ADDR 0x76

uint8_t bmp_read_reg(uint8_t reg) {
    uint8_t value = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR << 1, reg, 1, &value, sizeof(value), 10) != HAL_OK){ // TODO: non-blocking read
      Error_Handler();
    }
    return value;
}

HAL_StatusTypeDef bmp_heartbeat() {
    return HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR << 1, 1, 10);
}