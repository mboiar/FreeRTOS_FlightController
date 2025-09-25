#include "Tasks.h"
#include "tim.h"
#include "usart.h"
#include "w25q64.h"

// osThreadId_t TaskStartupHandle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  // if (GPIO_Pin == GPIO_PIN_0) {
  //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //     xTaskNotifyFromISR(defaultTaskHandle, 0x01, eSetBits,
  //     &xHigherPriorityTaskWoken);
  //     // vTaskNotifyGiveFromISR(defaultTaskHandle,
  //     &xHigherPriorityTaskWoken);
  //     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  // }
  // if (GPIO_Pin == GPIO_PIN_8) {
  //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //     xTaskNotifyFromISR(defaultTaskHandle, 0x04, eSetBits,
  //     &xHigherPriorityTaskWoken);
  //     // vTaskNotifyGiveFromISR(defaultTaskHandle,
  //     &xHigherPriorityTaskWoken);
  //     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  // }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    Flash_SPI_TxRxCpltHandler();
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    Flash_SPI_TxCpltHanlder();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    Logging_UART_TxCpltHandler();
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    Radio_UART_RxHalfCpltHandler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    Radio_UART_RxCpltHandler();
  }
}

void Flash_SPI_TxCpltHanlder() {
  w25q64_transfer_done();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(TaskStartupHandle, 0x02, eSetBits,
                     &xHigherPriorityTaskWoken);
  // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Flash_SPI_TxRxCpltHandler() {
  w25q64_transfer_done();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(TaskStartupHandle, 0x02, eSetBits,
                     &xHigherPriorityTaskWoken);
  // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void TIM3_IRQHandler(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint32_t tick = 0;

  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

    tick++;

    // 4 kHz
    if (tick % 1 == 0) {
      xTaskNotifyFromISR(TaskFlightLoopHandle, 0x01, eSetBits,
                         &xHigherPriorityTaskWoken);
      xTaskNotifyFromISR(TaskSensorHandle, 0x01, eSetBits,
                         &xHigherPriorityTaskWoken);
    }
    // 250 Hz
    if (tick % 16 == 0) {
      xTaskNotifyFromISR(TaskRadioRXHandle, 0x01, eSetBits,
                         &xHigherPriorityTaskWoken);
    }
    // 50 Hz
    if (tick % 80 == 0) {
      xTaskNotifyFromISR(TaskUARTLoggingHandle, 0x01, eSetBits,
                         &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}