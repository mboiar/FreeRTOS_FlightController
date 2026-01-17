// Driver for a generic ELRS radio RX
//
// Maks Boiar 2025
//

#pragma once

#include "stdint.h"
#include "string.h"

#define CRSF_TYPE_RC 0x16

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_TYPE_BATTERY 0x08
#define CRSF_TYPE_FLIGHT_MODE 0x21

#define RC_VAL_MAX 1811.0f
#define RC_VAL_MIN 174.0f
#define RC_VAL_MID ((RC_VAL_MAX + RC_VAL_MIN) / (float)2)
#define RC_RANGE (RC_VAL_MAX - RC_VAL_MIN)

#define RC_VAL8_MAX 1792.0f
#define RC_VAL8_MIN 191.0f
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

void crsf_pack_battery(uint8_t frame[16], uint16_t voltage_mv,
                       uint16_t current_ma, uint16_t capacity_mah,
                       uint8_t remaining_percent);

void crsf_pack_flight_mode(uint8_t frame[], char *flight_mode_str, int len);