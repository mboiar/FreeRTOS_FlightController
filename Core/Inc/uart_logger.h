#pragma once

#include "stm32f4xx_hal.h"
#include "usart.h"
// #include "stream_buffer.h"
// #include "stdlib.h"

#define LOG_BUFFER_SIZE 100

#define ACC_DP 1000
#define GYR_DP 1000


// typedef struct {
//     uint32_t timestamp;
//     uint16_t msg_len;
//     uint8_t  payload[];
// } __attribute__((packed)) log_entry_t;

HAL_StatusTypeDef log_write_uart(const uint8_t* pData, uint16_t size) {
    return HAL_UART_Transmit_IT(&huart1, pData, size);
}

int32_t ftoi(float x, uint32_t dp) {
    return (int32_t) (x*dp);
}

// int __io_putchar(int ch)
// {
//   if (ch == '\n') {
//     __io_putchar('\r');
//   }
//   log_write((uint8_t*)&ch, 1);
//   return 1;
// }