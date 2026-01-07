#pragma once

#define TASK_RADIORX_STACK_SIZE 0x600U
#define TASK_LOGGING_STACK_SIZE 0x800U
#define TASK_TELEM_STACK_SIZE 0x500U
#define TASK_SENSOR_STACK_SIZE 0x800U
#define TASK_FLIGHTLOOP_STACK_SIZE 0x600U
#define TASK_FAILSAFE_STACK_SIZE 0U
#define TASK_STARTUP_STACK_SIZE 0x500U
#define TASK_COMMRX_STACK_SIZE 0x400U

#define TASK_RADIORX_PRIORITY 5
#define TASK_LOGGING_PRIORITY 7
#define TASK_TELEM_PRIORITY 3
#define TASK_SENSOR_PRIORITY 3
#define TASK_FLIGHTLOOP_PRIORITY 2
#define TASK_FAILSAFE_PRIORITY 1
#define TASK_STARTUP_PRIORITY 3
#define TASK_COMMRX_PRIORITY 6

#define LOG_QUEUE_LEN 5

#define RADIORX_DMA_LEN 128
#define COMMRX_DMA_LEN 400

#define CRSF_BUFFER_SIZE 64
#define COMM_BUFFER_SIZE 256

#define RC_MAP_CH_ROLL 0
#define RC_MAP_CH_PITCH 1
#define RC_MAP_CH_YAW 3
#define RC_MAP_CH_THROTTLE 2
#define RC_MAP_CH_MODE 6
#define RC_MAP_CH_ARM 8
#define RC_MAP_CH_MODE1 5
#define RC_MAP_CH_MODE_CH_REQ 4

#define RC_ANGLE_MAX 30  // deg
#define RC_ANGLE_MIN -30 // deg
#define RC_VEL_MAX 15    // deg/s
#define RC_VEL_MIN -15   // deg/s
#define RC_VEL_RANGE (((float)RC_VEL_MAX - (float)RC_VEL_MIN))
#define RC_VEL_MID (((float)RC_VEL_MIN + (float)RC_VEL_MAX) / 2.0f)
#define RC_THROTTLE_MAX 0 //
#define RC_THROTTLE_MIN 1 //
#define RC_THROTTLE_RANGE (((float)RC_THROTTLE_MAX - (float)RC_THROTTLE_MIN))
#define RC_THROTTLE_MID                                                        \
  (((float)RC_THROTTLE_MIN + (float)RC_THROTTLE_MAX) / 2.0f)

#define HCSR04_SENSOR_COUNT 6
#define HCSR04_TRIG_PORT GPIOB
#define HCSR04_TRIG_PIN GPIO_PIN_12
#define HCSR04_BUFFER_LEN 3

#define ESKF_SAN 0.02f     // m/s^2
#define ESKF_SWN 0.002f    // rad/s
#define ESKF_SAW 0.02f     // m/s^2/sqrtHz
#define ESKF_SWW 0.00001f  // rad/s/sqrtHz
#define EKSF_SMAG 0.05     // ?
#define EKSF_SBARO 0.1     // m ?
#define ESKF_SGPS_VEL 0.05 // m/s
#define ESKF_SGPS_POS 2.5  // m

#define ENABLE_RUNTIME_STATS 1
#define PID_DEBUG 1
#define RADIO_DEBUG 0
#define ENABLE_HIL 1
#define ENABLE_GUIDED 1
