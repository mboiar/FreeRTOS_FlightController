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

#include "uart_logger.h"
#include "controller.h"

#include "stdio.h"
#include "usart.h"
#include "i2c.h"
#include "string.h"
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
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  /* Infinite loop */
  for(;;)
  {
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
  if (bmp_init(&tp, &pp, ctrl_p, conf_p) == HAL_OK) {
    printf("OK: BMP280");
  } else {
    printf("Error: BMP280 Init failed");
  }
  osDelay(100);
  float temp = 0, pressure = 0;
  for (;;) {
    printf("TaskMPU");
    bmp_acquire_data(&pressure, &temp, tp, pp);
    osDelay(500);
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

/* USER CODE END Application */

