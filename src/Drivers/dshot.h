/*
DSHOT protocol implementation for ESC communication.

Author: Maks Boiar

*/

#pragma once

#include <stdint.h>

typedef enum { DSHOT150 = 1, DSHOT300, DSHOT600 } dshot_type_t;

#define DSHOT_VAL_MIN 48
#define DSHOT_VAL_MAX 2047
#define DSHOT_VAL_MID ((DSHOT_VAL_MAX + DSHOT_VAL_MIN) / 2U)
#define DSHOT_RANGE (DSHOT_VAL_MAX - DSHOT_VAL_MIN)

#define dshotTYPE_TO_HZ(x) (uint32_t)(x * 150000U)

#define DSHOT_DMA_BUF_SIZE 18

// Initialize DSHOT communication
// @param dshot_type DSHOT frequency
void dshot_init(dshot_type_t dshot_type);

int dshot_write(uint16_t data, uint8_t telem, uint8_t ch);

int dshot_set_direction(uint8_t dir, uint8_t ch);