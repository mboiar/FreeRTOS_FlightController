/*
DSHOT protocol implementation for ESC communication.

Author: Maks Boiar

*/

#pragma once

// #include "stm32f4xx_hal.h"
#include "stdint.h"

typedef enum { DSHOT150 = 1, DSHOT300, DSHOT600 } dshot_type_t;

#define dshotTYPE_TO_HZ(x) (uint32_t)(x * 150000U)

#define DSHOT_DMA_BUF_SIZE 16

// Initialize DSHOT communication
// @param dshot_type DSHOT frequency
void dshot_init(dshot_type_t dshot_type);

int dshot_write(uint16_t data, uint8_t telem);
