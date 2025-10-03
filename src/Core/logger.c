#include "logger.h"
#include "Tasks.h"

static MAV_SEVERITY LOG_LEVEL_TASKS[TaskID_LEN] = {
    MAV_SEVERITY_DEBUG, MAV_SEVERITY_ERROR, MAV_SEVERITY_ERROR,
    MAV_SEVERITY_ERROR, MAV_SEVERITY_ERROR, MAV_SEVERITY_ERROR};

static void mavlink_log_pack(uint8_t src, MAV_SEVERITY severity,
                             mavlink_message_t *msg, const char *fmt, ...) {
  char buf[256] = {0};
  buf[0] = src;
  buf[1] = ":";
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf + 2, sizeof(buf), fmt, args);
  va_end(args);

  mavlink_msg_statustext_pack(1, MAV_COMP_ID_AUTOPILOT1, msg, severity, buf, 0,
                              0);
}

void mavlink_log_send(uint8_t src, MAV_SEVERITY severity, const char *fmt,
                      ...) {
  if (LOG_LEVEL_TASKS[src] < severity) {
    return;
  }
  mavlink_message_t msg;
  va_list args;
  va_start(args, fmt);
  mavlink_log_pack(src, severity, &msg, fmt, args);
  va_end(args);
  xQueueSend(xLogQueue, &msg, 0);
}

void comm_tx_send(void *pdata) { xQueueSendToBack(xLogQueue, pdata, 0); }
