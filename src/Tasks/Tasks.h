#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "logger.h"
#include "stream_buffer.h"
#include <task.h>

#define LOG_RADIO_RX 0

extern osThreadId_t TaskSensorHandle;
extern osThreadId_t TaskTelemetryHandle;
extern osThreadId_t TaskRadioRXHandle;
extern osThreadId_t TaskFlightLoopHandle;
extern osThreadId_t TaskUARTLoggingHandle;
extern osThreadId_t TaskStartupHandle;

extern StreamBufferHandle_t crsfStream;
extern state_t state;

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
