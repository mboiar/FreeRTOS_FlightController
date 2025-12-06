#include "utils.h"
#include "logger.h"

uint8_t I2C_Scan(I2C_HandleTypeDef *hi2c) {
  uint8_t i2c_dev_count = 0;
  for (uint8_t addr = 1; addr < 128; addr++) {
    if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, 10) == HAL_OK) {
      LOG_DEBUG(0, "I2C device found at 0x%02X\r\n", addr);
      i2c_dev_count++;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
  return i2c_dev_count;
}
