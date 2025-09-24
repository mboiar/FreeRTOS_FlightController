#include "logger.h"

void mavlink_log(MAV_SEVERITY severity, mavlink_message_t *msg, const char *fmt,
                 ...) {
  char buf[50] = {0};
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  mavlink_msg_statustext_pack(1, MAV_COMP_ID_AUTOPILOT1, msg, severity, buf, 0,
                              0);
}

void mavlink_log_send(MAV_SEVERITY severity, const char *fmt, ...) {
  mavlink_message_t msg;
  va_list args;
  va_start(args, fmt);
  mavlink_log(severity, &msg, fmt, args);
  va_end(args);
  xQueueSend(xLogQueue, &msg, 0);
}