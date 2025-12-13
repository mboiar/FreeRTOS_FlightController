#pragma once

#include "common/mavlink.h"

#define SENSOR_MEASURE 0x01
#define SENSOR_CALIBRATION_START (0x01 << 1)
#define SENSOR_CALIBRATION_STOP (0x01 << 2)
#define SENSOR_LOAD_PARAMS (0x01 << 3)
#define SENSOR_DEBUG_EKF (0x01 << 4)
#define SENSOR_FUSE_GPS (0x01 << 5)

#define RADIORX_REQUEST_RAW 0x01
#define RADIORX_REQUEST_SCALED (0x01 << 1)

#define TELEM_GET_RUNTIME_STATS (0x01 << 1)

#define PID_COMPUTE 0x01
#define PID_SET_TARGET_VELOCITY (0x01 << 1)

#define MAV_CMD_REQUEST_RC_RAW 12
#define MAV_CMD_REQUEST_RC_SCALED 15
#define MAV_CMD_REQUEST_EKF 13
#define MAV_CMD_REQUEST_RUNTIME_STATS 14

typedef struct {
  uint8_t type;
  uint8_t autopilot;
  uint16_t mode;
  uint16_t custom_mode;
  uint8_t state;
  uint8_t system_id;
  uint8_t comp_id;
  uint64_t home_lon;
  uint64_t home_lat;
  float home_alt;
} FC_State;
