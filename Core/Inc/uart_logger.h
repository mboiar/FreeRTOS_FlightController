#pragma once

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "stream_buffer.h"

#define LOG_BUFFER_SIZE 10

typedef struct {
    uint8_t data[LOG_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} LogBuffer;

HAL_StatusTypeDef log_write(const uint8_t* pData, uint16_t size) {
    return HAL_UART_Transmit_DMA(&huart1, pData, size);
}

int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }
  log_write((uint8_t*)&ch, 1);
  return 1;
}