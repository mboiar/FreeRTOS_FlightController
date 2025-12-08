#pragma once

#include "API.h"
#include "Config.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "crsf.h"
#include "hcsr04.h"
#include "imu.h"
#include "logger.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

#include <limits.h>

typedef enum {
  TASK_SENSOR_ID,
  TASK_RADIO_RX_ID,
  TASK_TELEMETRY_ID,
  TASK_FLIGHT_LOOP_ID,
  TASK_LOGGING_ID,
  TASK_STARTUP_ID,
  TASK_COMMRX_ID,
  TaskID_LEN
} TaskID;

extern FC_State fc_state;
extern GPS_data gps_data;

extern osThreadId_t TaskSensorHandle;
extern osThreadId_t TaskTelemetryHandle;
extern osThreadId_t TaskRadioRXHandle;
extern osThreadId_t TaskFlightLoopHandle;
extern osThreadId_t TaskUARTLoggingHandle;
extern osThreadId_t TaskStartupHandle;
extern osThreadId_t TaskCommRxHandle;

extern StreamBufferHandle_t crsfStream;
extern StreamBufferHandle_t commRXStream;
extern state_t state;
extern SemaphoreHandle_t imu_mutex;

// extern uint16_t HCSR04_ECHO_PIN[HCSR04_SENSOR_COUNT];
// extern GPIO_TypeDef *HCSR04_ECHO_PORT[HCSR04_SENSOR_COUNT];
// extern hcsr04_sensor_t sensors[HCSR04_SENSOR_COUNT];

extern crsf_rc_t rc_data;

extern float magcal_offset[3];
extern float magcal_mat[3][3];
extern float mag_decl;
extern float mag_incl;
extern accel3d_t offA;
extern float scaleA[3];
extern sensor_data_t imu_data;

void StartupTask(void *argument);
void TaskSensor(void *argument);
void TaskRadioRX(void *arg);
void TaskFlightLoop(void *argument);
void TaskUARTLogging(void *argument);
void TaskTelemetry(void *arg);
void TaskCommRx(void *arg);

void Radio_UART_RxHalfCpltHandler();
void Radio_UART_RxCpltHandler();
void CommRx_UART_RxHalfCpltHandler();
void CommRx_UART_RxCpltHandler();
void Logging_UART_TxCpltHandler();
void Flash_SPI_TxCpltHanlder();
void Flash_SPI_TxRxCpltHandler();
void TIM3_TaskNotifyISR();
void CommRx_UARTEx_RxEventHandler(uint16_t Size);
void DistanceSensor_RxCpltCallback(uint16_t GPIO_Pin);