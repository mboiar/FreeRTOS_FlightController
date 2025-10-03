#include "Tasks.h"
#include "usart.h"

// osThreadId_t TaskUARTLoggingHandle;

/**
 * @brief Logs messages to serial port
 * @param argument: Not used
 * @retval None
 */
void TaskUARTLogging(void *argument) {
  uint32_t ulNotifiedValue;
  HAL_StatusTypeDef UARTStatus;
  BaseType_t xQueueStatus;
  BaseType_t xResult;
  uint8_t data_buf[BUFFER_SIZE] = {0};
  // uint8_t packet[PACKET_SIZE] = {0};
  size_t length;
  TickType_t timestamp;
  mavlink_message_t msg;
  for (;;) {
    // memset(data_buf, 0, sizeof(data_buf));
    xQueueStatus = xQueueReceive(xLogQueue, &msg, portMAX_DELAY);
    length = mavlink_msg_to_send_buffer(data_buf, &msg);
    UARTStatus = HAL_UART_Transmit_DMA(&huart1, data_buf, length);
    // Do NOT start another send until previous completed
    xResult =
        xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
  }
}

void Logging_UART_TxCpltHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(TaskUARTLoggingHandle, 0x01, eSetBits,
                     &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
