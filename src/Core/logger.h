#pragma once

// #include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "common/mavlink.h"
#include "stdarg.h"

#define BUFFER_SIZE MAVLINK_MAX_PACKET_LEN
#define PACKET_SIZE 128

typedef struct {
  uint8_t sysid;
  mavlink_control_system_state_t ctrl_sys_state;
  uint8_t mode;
  uint8_t sys_status;
} state_t;

extern QueueHandle_t xLogQueue;

void mavlink_log(MAV_SEVERITY severity, mavlink_message_t *msg, const char *fmt,
                 ...);
void mavlink_log_send(MAV_SEVERITY severity, const char *fmt, ...);

#define LOG_DEBUG(fmt, ...)                                                    \
  mavlink_log_send(MAV_SEVERITY_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)                                                     \
  mavlink_log_send(MAV_SEVERITY_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)                                                     \
  mavlink_log_send(MAV_SEVERITY_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)                                                      \
  mavlink_log_send(MAV_SEVERITY_ERROR, fmt, ##__VA_ARGS__)
#define LOG_CRIT(fmt, ...)                                                     \
  mavlink_log_send(MAV_SEVERITY_CRITICAL, fmt, ##__VA_ARGS__)
