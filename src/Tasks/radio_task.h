#pragma once

#include "FreeRTOS.h"
#include "stream_buffer.h"

#define LOG_RADIO_RX 0

extern StreamBufferHandle_t crsfStream;


void TaskRadioRX(void *arg);

void Radio_UART_RxHalfCpltHandler();

void Radio_UART_RxCpltHandler();