#pragma once

#include "i2c.h"

/**
 * @brief Lists available I2C devices
 * @param hi2c i2c handle
 * @retval None
 */
void I2C_Scan(I2C_HandleTypeDef *hi2c);
