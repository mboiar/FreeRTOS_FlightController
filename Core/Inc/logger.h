
#include "stm32f4xx.h"

#include <mavlink/v2.0/common/mavlink.h>
#include <stdarg.h>


#define BUFFER_SIZE MAVLINK_MAX_PACKET_LEN
#define PACKET_SIZE 128

typedef enum {
    MSG_DEBUG,
    MSG_INFO,
    MSG_ERROR,
    MSG_CRITICAL,
    DATA_SENSORS,
    DATA_DEBUG,
    DATA_CTRL
} LOG_TYPE;

typedef struct {
    LOG_TYPE lvl;
    uint8_t data[BUFFER_SIZE];
} log_msg_t;

void mavlink_log(MAV_SEVERITY severity, mavlink_message_t* msg, const char *fmt, ...) {
    char buf[50] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    mavlink_msg_statustext_pack(
      1,
      MAV_COMP_ID_AUTOPILOT1,
      &msg,
      severity,
      buf,
      0, 0
    );
}