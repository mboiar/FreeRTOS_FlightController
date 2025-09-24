#pragma once

// #include "stm32f4xx.h"
#include "FreeRTOS.h"
// #include "queue.h"

// #include "common/mavlink.h"
#include "stdarg.h"

#define BUFFER_SIZE 256 //MAVLINK_MAX_PACKET_LEN
#define PACKET_SIZE 128

extern QueueHandle_t xLogQueue;


void mavlink_log(MAV_SEVERITY severity, mavlink_message_t* msg, const char *fmt, ...) {
    char buf[50] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    mavlink_msg_statustext_pack(
      1,
      MAV_COMP_ID_AUTOPILOT1,
      msg,
      severity,
      buf,
      0, 0
    );
}

void mavlink_log_send(MAV_SEVERITY severity, const char *fmt, ...) {
    mavlink_message_t msg;
    va_list args;
    mavlink_log(severity, &msg, fmt, args);
    xQueueSend(xLogQueue, &msg, 0);
}

#define LOG_DEBUG(fmt, ...) mavlink_log_send(MAV_SEVERITY_DEBUG, fmt, ...)
#define LOG_INFO(fmt, ...) mavlink_log_send(MAV_SEVERITY_INFO, fmt, ...)
#define LOG_WARN(fmt, ...) mavlink_log_send(MAV_SEVERITY_WARNING, fmt, ...)
#define LOG_ERR(fmt, ...) mavlink_log_send(MAV_SEVERITY_ERROR, fmt, ...)
#define LOG_CRIT(fmt, ...) mavlink_log_send(MAV_SEVERITY_CRITICAL, fmt, ...)
