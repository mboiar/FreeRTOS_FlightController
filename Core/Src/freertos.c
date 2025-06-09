/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "bmp280.h"
#include "flash_mem.h"

#include "uart_logger.h"
#include "controller.h"

#include "stdio.h"
#include "usart.h"
#include "i2c.h"
#include "string.h"

#include "melody.h"

#include "limits.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t TaskMPUHandle;
const osThreadAttr_t TaskMPU_attributes = {
  .name = "TaskMPU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t TaskFlightLoopHandle;
const osThreadAttr_t TaskFlightLoop_attributes = {
  .name = "TaskFlightLoop",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};

osThreadId_t TaskUARTLoggingHandle;
const osThreadAttr_t TaskUARTLogging_attributes = {
  .name = "TaskUARTLogging",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};

const UBaseType_t xArrayIndex = 1;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void TaskMPU(void *argument);
void TaskFlightLoop(void *argument);
void TaskUARTLogging(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
   
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  TaskMPUHandle = osThreadNew(TaskMPU, NULL, &TaskMPU_attributes);
  // TaskFlightLoopHandle = osThreadNew(TaskFlightLoop, NULL, &TaskFlightLoop_attributes);
  // TaskUARTHandle = osThreadNew(TaskUARTLogging, NULL, &TaskUARTLogging_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  //uint32_t PreviousWakeTime = osKernelSysTick();
  //uint32_t OctaveFour[7] = {262, 294, 330, 349, 392, 440, 493}; // Octave_4 Frequencies (C4->B4)
  uint8_t NoteIndex = 0;

  // Tone cur_tone;
  static UBaseType_t blocked = 1;

  uint8_t resp[6] = {0};
  uint8_t flashmem_device_id_val = 0, flashmem_manuf_id_val = 0;
  flashmem_device_id(resp);
  // ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
  uint32_t ulNotifiedValue = 0;
  BaseType_t xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
  if (xResult == pdPASS) {
    if ((ulNotifiedValue & 0x02) != 0) {
      flashmem_manuf_id_val = resp[4];
      flashmem_device_id_val = resp[5];
    }
  } else {
    // FAIL
  }
  uint8_t status_bits[3] = {0};
  flashmem_status(status_bits);
  xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
  if ((ulNotifiedValue & 0x02) != 0) {

  }
  flashmem_sector_erase(0, SECTOR_ERASE_4K);
  uint8_t data[15] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x08, 0x04, 0x02, 0x01, 0x01, 0x01, 0x08, 0x04, 0x02};
  flashmem_page_program(data, 15, 0);
  xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
  if ((ulNotifiedValue & 0x02) != 0) {

  }
  uint8_t data_rxtx[14] = {0};
  flashmem_read_data(data_rxtx, 14, 0);
  xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
  if ((ulNotifiedValue & 0x02) != 0) {

  }

  /* Infinite loop */
  for(;;) {
    if (blocked) {
      // if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) == pdTRUE) {
      xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
      if (xResult == pdPASS) {
        if ((ulNotifiedValue & 0x01) != 0) {
          blocked = 0;
        }
      }
    } else {
      // if (ulTaskNotifyTake(pdFALSE, 0)) {
      xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, 0);
      if (xResult == pdPASS) {
        if ((ulNotifiedValue & 0x01) != 0) {
          blocked = 1;
          TIM1->CCR1 = 0;
          continue;
        }
      }
    }
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    if(NoteIndex == 25){
      NoteIndex = 0;
    }
    int freq = hb_melody[NoteIndex];
    int dur = hb_durations[NoteIndex];
    // cur_tone = canon_melody[NoteIndex];
    if (freq > 0) {
      TIM1->ARR = (1000000UL / freq) - 1; // Set The PWM Frequency
      TIM1->CCR1 = (TIM1->ARR >> 1); // Set Duty Cycle 50%
    } else {
      TIM1->CCR1 = 0;
    }
    NoteIndex++;
    vTaskDelay(pdMS_TO_TICKS(1000 / dur));
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Task to handle MPU operations
 * @param argument: Not used
 * @retval None
 */
void TaskMPU(void *argument) {

  I2C_Scan(&hi2c1);
  BMP_CONFIG_PARAMS conf_p = {
    .filter_coef = 4,           // x16
    .standby_time = 0,           // 0.5 ms
    .spi3w_en = 0
  };
  BMP_CTRL_MEAS_PARAMS ctrl_p = {
    .mode = BMP_FORCED,
    .temp_oversampling = 1,     // x1
    .pressure_oversampling = 3, // x4
  };
  BMP_CAL_T_PARAMS tp; BMP_CAL_P_PARAMS pp;
  // if (bmp_init(&tp, &pp, ctrl_p, conf_p) == HAL_OK) {
  //   printf("OK: BMP280");
  // } else {
  //   printf("Error: BMP280 Init failed");
  // }
  // osDelayUntil(100);
  // float temp = 0, pressure = 0;
  for (;;) {
  //   printf("TaskMPU");
  //   bmp_acquire_data(&pressure, &temp, tp, pp);
    vTaskDelay(2000);
  }
}

/**
 * @brief Logs telemetry to serial port
 * @param argument: Not used
 * @retval None
 */
void TaskUARTLogging(void *argument) {
  for (;;) {

  }
}

/**
 * @brief Flight Loop
 * @param argument: Not used
 * @retval None
 */
void TaskFlightLoop(void *argument) {
  for (;;) {

  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(defaultTaskHandle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
        // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    flashmem_transfer_done();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(defaultTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
    // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    flashmem_transfer_done();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(defaultTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
    // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
/* USER CODE END Application */

