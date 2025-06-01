#pragma once

#include "stm32f4xx.h"


/* Read MPU6050 register in blocking mode */
uint8_t mpu_read_reg(uint8_t reg);

/* Write MPU6050 register */
uint8_t mpu_write_reg(uint8_t reg, const void* data);

/* Check MPU6050 status*/
HAL_StatusTypeDef mpu_heartbeat();
