// Driver for a generic ELRS radio RX
//
// Maks Boiar 2025
//

#pragma once

#include "stdint.h"
#include "string.h"

#define CRSF_TYPE_RC 0x16

#define RC_VAL_MAX (float)1811
#define RC_VAL_MIN (float)174
#define RC_VAL_MID ((RC_VAL_MAX + RC_VAL_MIN) / (float)2)
#define RC_RANGE (RC_VAL_MAX - RC_VAL_MIN)

#define RC_VAL8_MAX (float)1792
#define RC_VAL8_MIN (float)191
#define RC_VAL8_MID ((RC_VAL8_MAX + RC_VAL8_MIN) / (float)2)
#define RC_RANGE8 (RC_VAL8_MAX - RC_VAL8_MIN)

#define CRSF_MAX_FRAME_LEN 64

typedef struct {
  uint16_t ch_data[16];
} crsf_rc_t;

typedef struct {
  uint8_t addr, len, type;
  uint8_t payload[CRSF_MAX_FRAME_LEN - 4];
  uint8_t idx;
} crsf_frame_t;

typedef enum {
  CRSF_ADDR,
  CRSF_LEN,
  CRSF_TYPE,
  CRSF_PAYLOAD,
  CRSF_CRC
} crsf_state_t;

int radio_parse_crsf_byte(crsf_frame_t *frame, uint8_t byte,
                          crsf_state_t *state);

void radio_unpack_rc(crsf_rc_t *rc_data, const uint8_t *rc_data_raw);
