#pragma once

#include "common/mavlink.h"

#define SENSOR_MEASURE 1UL
#define SENSOR_CALIBRATION_START (1UL << 1)
#define SENSOR_CALIBRATION_STOP (1UL << 2)
#define SENSOR_LOAD_PARAMS (1UL << 3)
#define SENSOR_DEBUG_EKF (1UL << 4)
#define SENSOR_FUSE_GPS (1UL << 5)

#define RADIORX_REQUEST_RAW 1UL
#define RADIORX_REQUEST_SCALED (1UL << 1)

#define TELEM_GET_RUNTIME_STATS (1UL << 1)

#define PID_COMPUTE 1UL
#define PID_SET_TARGET_VELOCITY (1UL << 1)
#define PID_ARM_READY (1UL << 2)

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
  uint32_t sensors_detected; // TODO
  uint32_t sensors_health;   // TODO
  uint32_t sensors_enabled;
  int32_t home_lon;
  int32_t home_lat;
  float home_alt;
  uint8_t battery_state;
  uint16_t battery_voltage;
} FC_State;
