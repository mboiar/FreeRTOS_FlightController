
#include "stm32f4xx.h"


#define BUFFER_SIZE 256
#define PACKET_SIZE 32

typedef enum {
    MSG_DEBUG,
    MSG_INFO,
    MSG_ERROR,
    MSG_CRITICAL,
    DATA_SENSORS,
    DATA_CTRL
} LOG_TYPE;

typedef struct {
    LOG_TYPE lvl;
    uint8_t data[BUFFER_SIZE];
} log_msg_t;
