#include "utils.h"

void I2C_Scan(I2C_HandleTypeDef *hi2c) {
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, 10) == HAL_OK) {
            printf("I2C device found at 0x%02X\r\n", addr);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}
