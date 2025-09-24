#pragma once

// #include "mpu6050.h"
// #include "bmp280.h"
// #include "w25q64.h"
// #include "qmc5883.h"
// #include "crsf.h"
// #include "logger.h"
// #include "utils.h"
#include "Tasks.h"

// #include "string.h"
// #include "limits.h"
// #include "queue.h"

// #include "FreeRTOS.h"
// #include "task.h"
// #include "main.h"
// #include "cmsis_os.h"


typedef struct {
  uint8_t sysid;
  mavlink_control_system_state_t ctrl_sys_state;
  uint8_t mode;
  uint8_t sys_status;
} state_t;


void Init();
