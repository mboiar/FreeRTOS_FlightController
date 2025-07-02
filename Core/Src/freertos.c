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
#include "w25q64.h"
#include "qmc5883.h"
// #include "l76.h"
#include "motor_control.h"
#include "logger.h"

#include "uart_logger.h"
//#include "controller.h"

//#include "stdio.h"
#include "usart.h"
#include "i2c.h"
#include "string.h"

//#include "melody.h"

#include "limits.h"
#include "queue.h"
// #include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  accel_3d accel;
  gyro_3d gyro;
  float heading;
  float alt;
  TickType_t timestamp;
  int16_t bmp_temp;
} sensor_data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t TaskSensorHandle;
const osThreadAttr_t TaskSensor_attributes = {
  .name = "TaskSensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};

osThreadId_t TaskGPSHandle;
const osThreadAttr_t TaskGPS_attributes = {
  .name = "TaskGPS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
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
  .priority = (osPriority_t) osPriorityHigh,
};

const UBaseType_t xArrayIndex = 1;

const size_t xQueueLen = 5;

QueueHandle_t xLogQueue;
uint8_t SensorDataBuffer[BUFFER_SIZE] = {0};
char DefaultTaskLog[50] = {0};

// uint8_t l76_buf[L76_MAX_BUF_SIZE] = {0};


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

void TaskSensor(void *argument);
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
  xLogQueue = xQueueCreate(xQueueLen, LOG_BUFFER_SIZE);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  TaskSensorHandle = osThreadNew(TaskSensor, NULL, &TaskSensor_attributes);

  // TaskFlightLoopHandle = osThreadNew(TaskFlightLoop, NULL, &TaskFlightLoop_attributes);
    if (xLogQueue != NULL) {
        TaskUARTLoggingHandle = osThreadNew(TaskUARTLogging, NULL, &TaskUARTLogging_attributes);
  }

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

  device_info device_info;
  w25q64_device_info(&device_info);
  
  status_registers status_bits;
  w25q64_status(&status_bits);

  // uint8_t data[14] = {0x00, 0x00, 0x00, 0x00, 0x10, 0x11, 0x13, 0x16, 0x1A, 0xB4, 0xD3, 0xF1, 0xDD, 0xAA};
  // w25q64_page_program_IT(data, 14, 0x33);
  // uint32_t ulNotifiedValue = 0;
  // BaseType_t xResult;
  // // while ((ulNotifiedValue & 0x02) == 0) {
  //   xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
  // }
  bool busy = true;
  BaseType_t xStatus;
  while (busy) {
    w25q64_isbusy(&busy);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  uint8_t data_rxtx1[14] = {0};
  w25q64_read_data(data_rxtx1, 14, 0x33);
  // memcpy(DefaultTaskLog, "Default task is called here   \r\n", 33);

  bool melody_completed = false;
  // vTaskSuspend(NULL);

  /* Infinite loop */
  for(;;) {
    xStatus = xQueueSend(xLogQueue, DefaultTaskLog, 0);
    if (xStatus != pdPASS) {
        // handle queue fail
    }
    // if (blocked) {
    //   // if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) == pdTRUE) {
    //   xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
    //   if (xResult == pdPASS) {
    //     if ((ulNotifiedValue & 0x01) != 0) {
    //       blocked = 0;
    //     }
    //   }
    // } else {
    //   // if (ulTaskNotifyTake(pdFALSE, 0)) {
    //   xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, 0);
    //   if (xResult == pdPASS) {
    //     if ((ulNotifiedValue & 0x01) != 0) {
    //       blocked = 1;
    //       TIM1->CCR1 = 0;
    //       continue;
    //     }
    //   }
    // }
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if(NoteIndex == 25){
    //   NoteIndex = 0;
        melody_completed = true;
        // vTaskDelay(pdMS_TO_TICKS(2000));
        vTaskSuspend(NULL);
    } else {
        int freq = 330;// melody[NoteIndex];
        int dur = 4;//durations[NoteIndex];
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
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Task to handle sensor operations
 * @param argument: Not used
 * @retval None
 */
void TaskSensor(void *argument) {

  I2C_Scan(&hi2c1);

  HAL_StatusTypeDef mpu_status = mpu6050_heartbeat();
  mpu6050_set_power_options(CLKSEL_PLLX, 0);
  mpu6050_set_config(MPU6050_I2C_BYPASS_EN, MPU6050_DATA_RDY_EN);

  mpu6050_out mpu6050_data;
  float mpu_temp;
  BMP_CAL_T_PARAMS tp; BMP_CAL_P_PARAMS pp;
  BaseType_t queue_status;

  char SensorLog[LOG_BUFFER_SIZE] = {0};

    BMP_CONFIG_PARAMS BMP280_CONFIG_DEFAULT = {
        .filter_coef = 4,           // x16
        .standby_time = 0,           // 0.5 ms
        .spi3w_en = 0
    };

    BMP_CTRL_MEAS_PARAMS BMP280_CTRL_MEAS_DEFAULT = {
        .mode = BMP_NORMAL,
        .temp_oversampling = 1,     // x1
        .pressure_oversampling = 3, // x4
    };
  HAL_StatusTypeDef bmp_status = bmp_init(&tp, &pp, BMP280_CTRL_MEAS_DEFAULT, BMP280_CONFIG_DEFAULT);
  float bmp_temp = 0, bmp_pressure = 0;

  // get reference pressure
  bmp_acquire_data(&bmp_pressure, &bmp_temp, tp, pp);
  const float p_ref = bmp_pressure;

  // Configure QMC5883 magnetometer
  HAL_StatusTypeDef qmc_status = qmc5883_heartbeat();
  qmc5883_set_config(QMC5883_CONTINUOUS | ODR_100HZ | RNG_2G | OSR_512);
  qmc5883_set_ctrl(INT_DISABLE | ROL_PNT_NORMAL);
  qmc5883_out qmc5883_data;

  // Redirect mag data to mpu6050 for sensor sync
  mpu6050_set_master_ctrl(MPU6050_WAIT_FOR_ES);
  mpu6050_set_config(0, MPU6050_DATA_RDY_EN);
  mpu6050_user_ctrl(MPU6050_I2C_MST_EN);

  qmc_status = mpu6050_slv0_init();

  sensor_data_t sdata;

  for (;;) {
    mpu6050_read_data(&mpu6050_data, &qmc5883_data);                    // blocking ? TODO
    bmp_acquire_data(&bmp_pressure, &bmp_temp, tp, pp);  // blocking ?                TODO

    sdata.alt = bmp280_get_altitude(bmp_pressure, p_ref, sdata.bmp_temp);
    // qmc5883_read_data(&qmc5883_data);
    sdata.heading = qmc5883_get_heading(&qmc5883_data, 108.8 / 1000.0);
    mpu_temp = mpu6050_calc_temp(mpu6050_data.temp);
    sdata.accel.accel_x = mpu6050_calc_accel(mpu6050_data.accel_x, ACCEL_FS_2G);
    sdata.accel.accel_y = mpu6050_calc_accel(mpu6050_data.accel_y, ACCEL_FS_2G);
    sdata.accel.accel_z = mpu6050_calc_accel(mpu6050_data.accel_z, ACCEL_FS_2G);
    sdata.gyro.gyro_x = mpu6050_calc_accel(mpu6050_data.gyro_x, FS_SEL_250);
    sdata.gyro.gyro_y = mpu6050_calc_accel(mpu6050_data.gyro_y, FS_SEL_250);
    sdata.gyro.gyro_z = mpu6050_calc_accel(mpu6050_data.gyro_z, FS_SEL_250);

    sdata.timestamp = pdMS_TO_TICKS(xTaskGetTickCount());


    // snprintf(SensorLog, sizeof(SensorLog), "%lu %ld %ld %ld %ld %ld %ld %d %d %lu %d %d %d %d\r\n", timestamp, ftoi(accel.accel_x, ACC_DP), ftoi(accel.accel_y, ACC_DP), ftoi(accel.accel_z, ACC_DP), ftoi(gyro.gyro_x, GYR_DP), ftoi(gyro.gyro_y, GYR_DP), ftoi(gyro.gyro_z, GYR_DP), (int16_t)mpu_temp, (int16_t)bmp_temp, (uint32_t)(alt*100.0), qmc5883_data.MagX, qmc5883_data.MagY, qmc5883_data.MagZ, (int16_t)heading);
    SensorLog[0] = sizeof(sdata);
    SensorLog[1] = DATA_SENSORS;
    memcpy(SensorLog+2, &sdata, sizeof(sdata));
    queue_status = xQueueSend(xLogQueue, &SensorLog, 0);
    // if (queue_status != pdPASS) {
    //     // handle queue full
    // }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


/**
 * @brief Logs telemetry to serial port
 * @param argument: Not used
 * @retval None
 */
void TaskUARTLogging(void *argument) {
    uint32_t ulNotifiedValue;
    HAL_StatusTypeDef UARTStatus;
    BaseType_t xQueueStatus;
    BaseType_t xResult;
    uint8_t data_buf[LOG_BUFFER_SIZE];
    uint8_t packet[PACKET_SIZE];
    size_t length;
    for (;;) {
        xQueueStatus = xQueueReceive( xLogQueue, data_buf, portMAX_DELAY);
        length = data_buf[0];
        packet[0] = '$';
        packet[1] = data_buf[1];
        packet[30] = '\r';
        packet[31] = '\n';
        for (size_t i=0; i<length; i+=PACKET_SIZE-4) {
          memcpy(packet+2, data_buf+i+2, 28);
          UARTStatus = log_write_uart(packet, PACKET_SIZE);
          xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);  // Wait for UART tx to complete
        }
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
        if (GPIO_Pin == GPIO_PIN_8) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(defaultTaskHandle, 0x04, eSetBits, &xHigherPriorityTaskWoken);
        // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    w25q64_transfer_done();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(defaultTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
    // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    w25q64_transfer_done();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(defaultTaskHandle, 0x02, eSetBits, &xHigherPriorityTaskWoken);
    // vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(TaskUARTLoggingHandle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* USER CODE END Application */

