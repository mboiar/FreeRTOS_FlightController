#pragma once

#include "FreeRTOS.h"
#include "queue.h"

#include "common/mavlink.h"
#include <stdarg.h>

#define BUFFER_SIZE MAVLINK_MAX_PACKET_LEN
#define PACKET_SIZE 128

typedef struct {
  uint8_t sysid;
  mavlink_control_system_state_t ctrl_sys_state;
  uint8_t mode;
  uint8_t sys_status;
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
