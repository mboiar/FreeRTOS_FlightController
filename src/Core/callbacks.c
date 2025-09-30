#include "Tasks.h"
#include "mpu6050.h"
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

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c1) {
    IMU_RxCpltCallback();
  }
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

void TIM3_TaskNotifyISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint32_t tick = 0;

  tick++;

  // 4 kHz
  // xTaskNotifyFromISR(TaskFlightLoopHandle, 0x01, eSetBits,
  //                    &xHigherPriorityTaskWoken);

  // 1 kHz
  if (tick % 800 == 0) {
    xTaskNotifyFromISR(TaskSensorHandle, 0x01, eSetBits,
                       &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}