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
    // UARTStatus = HAL_UART_Transmit_DMA(&huart1, &msg, sizeof(msg));
    // timestamp = pdMS_TO_TICKS(xTaskGetTickCount());
    // length = data_buf[0];
    // packet[0] = 0x24;
    // packet[1] = 0x55;
    // packet[2] = data_buf[1]; // TYPE
    // size_t len = snprintf(packet+3, PACKET_SIZE, " %lu ", timestamp);
    // packet[PACKET_SIZE-2] = '\r';
    // packet[PACKET_SIZE-1] = '\n';
    // for (size_t i=0; i<length; i+=PACKET_SIZE-6-len) {
    //   memcpy(packet+4+len, data_buf+i+2, PACKET_SIZE-6-len);
    //   UARTStatus = HAL_UART_Transmit_DMA(&huart1, packet, PACKET_SIZE);
    //   xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue,
    //   portMAX_DELAY);  // Wait for UART tx to complete
    // }
  }
}

void Logging_UART_TxCpltHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(TaskUARTLoggingHandle, 0x01, eSetBits,
                     &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
