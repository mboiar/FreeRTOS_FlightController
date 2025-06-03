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
void I2C_Scan(I2C_HandleTypeDef *hi2c);
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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  TaskMPUHandle = osThreadNew(TaskMPU, NULL, &TaskMPU_attributes);
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
    printf("DefaultTask");
    osDelay(1000);
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
    .mode = BMP_NORMAL,
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

void I2C_Scan(I2C_HandleTypeDef *hi2c) {
    char msg[32];
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 3, 10) == HAL_OK) {
            sprintf(msg, "I2C device found at 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
    }
}
/* USER CODE END Application */

