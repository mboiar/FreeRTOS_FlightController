#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "imu.h"
#include "logger.h"
#include "stream_buffer.h"
#include <task.h>

typedef enum {
  TASK_SENSOR_ID,
  TASK_RADIO_RX_ID,
  TASK_TELEMETRY_ID,
  TASK_FLIGHT_LOOP_ID,
  TASK_LOGGING_ID,
  TASK_STARTUP_ID,
  TaskID_LEN
} TaskID;

extern osThreadId_t TaskSensorHandle;
extern osThreadId_t TaskTelemetryHandle;
extern osThreadId_t TaskRadioRXHandle;
extern osThreadId_t TaskFlightLoopHandle;
extern osThreadId_t TaskUARTLoggingHandle;
extern osThreadId_t TaskStartupHandle;

extern StreamBufferHandle_t crsfStream;
extern state_t state;
extern SemaphoreHandle_t imu_mutex;

void StartupTask(void *argument);
void TaskSensor(void *argument);
void TaskRadioRX(void *arg);
void TaskFlightLoop(void *argument);
void TaskUARTLogging(void *argument);
void TaskTelemetry(void *arg);

void Radio_UART_RxHalfCpltHandler();
void Radio_UART_RxCpltHandler();
void Logging_UART_TxCpltHandler();
void Flash_SPI_TxCpltHanlder();
void Flash_SPI_TxRxCpltHandler();
void TIM3_TaskNotifyISR();