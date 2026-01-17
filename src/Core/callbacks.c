#include "Tasks.h"
#include "mpu6050.h"
#include "stm32f4xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include "w25q64.h"

uint32_t TIM3tick = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  DistanceSensor_RxCpltCallback(GPIO_Pin);
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
  if (huart == &huart2) {
    Telem_UART_TxCpltHandler();
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    Radio_UART_RxHalfCpltHandler();
  } else if (huart == &huart1) {
    CommRx_UART_RxHalfCpltHandler();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    Radio_UART_RxCpltHandler();
  } else if (huart == &huart1) {
    CommRx_UART_RxCpltHandler();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart1) {
    CommRx_UARTEx_RxEventHandler(Size);
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

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // task scheduling
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

    TIM3tick++;
    // 1000 Hz
    if (TIM3tick % 1 == 0) {
      if (TaskSensorHandle != NULL) {
        xTaskNotifyFromISR(TaskSensorHandle, 0x01, eSetBits,
                           &xHigherPriorityTaskWoken);
      }
    }
    if (TIM3tick % 2 == 0) {
      if (TaskFlightLoopHandle != NULL) {
        xTaskNotifyFromISR(TaskFlightLoopHandle, 0x01, eSetBits,
                           &xHigherPriorityTaskWoken);
      }
    }
    // 10 Hz
    if (TIM3tick % 100 == 0) {
      if (TaskStartupHandle != NULL) {
        xTaskNotifyFromISR(TaskStartupHandle, 0x01, eSetBits,
                           &xHigherPriorityTaskWoken);
      }
    }

    TIM3->CCR1 = TIM3->CNT + 1000;
  } else if (htim->Instance == TIM3 &&
             htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    // hcsr04 callback
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_2);
    xTaskNotifyFromISR(TaskStartupHandle, 0x02, eSetBits,
                       &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
