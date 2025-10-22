#include "Tasks.h"
#include "main.h"
#include "usart.h"

static uint8_t data_buf[BUFFER_SIZE] = {0};

/**
 * @brief Logs messages to serial port
 * @param argument: Not used
 * @retval None
 */
void TaskUARTLogging(void *argument) {
  uint32_t ulNotifiedValue;
  size_t length;
  mavlink_message_t msg;

  for (;;) {
    xQueueReceive(xLogQueue, &msg, portMAX_DELAY);
    length = mavlink_msg_to_send_buffer(data_buf, &msg);
    if (HAL_UART_Transmit_DMA(&huart1, data_buf, length) != HAL_OK) {
      Error_Handler();
    };

    // Do NOT start another send until previous completed
    xTaskNotifyWait(pdFALSE, 0x01, &ulNotifiedValue, portMAX_DELAY);
  }
}

void Logging_UART_TxCpltHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(TaskUARTLoggingHandle, 0x01, eSetBits,
                     &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
