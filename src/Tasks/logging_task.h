#pragma once

#include "logger.h"
#include "FreeRTOS.h"


void TaskUARTLogging(void *argument);

void Logging_UART_TxCpltHandler();
