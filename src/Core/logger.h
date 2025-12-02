#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "common/mavlink.h"
#include <stdarg.h>

#define BUFFER_SIZE MAVLINK_MAX_PACKET_LEN
#define PACKET_SIZE 128

typedef enum {
  FLIGHT_MODE_LOITER,
  FLIGHT_MODE_AUTO,
  FLIGHT_MODE_GUIDED,
  FLIGHT_MODE_RTL,
  FLIGHT_MODE_LAND,
  FLIGHT_MODE_ALTHOLD
} FLIGHT_MODE;

typedef enum { LOCKED, MANUAL, GUIDED } SYS_MODE;

typedef struct {
  uint8_t sysid;
  mavlink_control_system_state_t ctrl_sys_state;
  FLIGHT_MODE custom_mode;
  SYS_MODE sys_mode;
  uint8_t nav_mode;
} state_t;

extern QueueHandle_t xLogQueue;

void comm_tx_send(void *pdata);

void mavlink_log_send(uint8_t src, MAV_SEVERITY severity, const char *fmt, ...);

#define LOG_DEBUG(src, fmt, ...)                                               \
  mavlink_log_send(src, MAV_SEVERITY_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(src, fmt, ...)                                                \
  mavlink_log_send(src, MAV_SEVERITY_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(src, fmt, ...)                                                \
  mavlink_log_send(src, MAV_SEVERITY_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERR(src, fmt, ...)                                                 \
  mavlink_log_send(src, MAV_SEVERITY_ERROR, fmt, ##__VA_ARGS__)
#define LOG_CRIT(src, fmt, ...)                                                \
  mavlink_log_send(src, MAV_SEVERITY_CRITICAL, fmt, ##__VA_ARGS__)
