#include "Program.h"
#include "COnfig.h"
#include "FreeRTOS.h"
#include "Tasks.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "logger.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "tim.h"

#include "Config.h"
#include "usart.h"

const osThreadAttr_t TaskSensor_attributes = {
    .name = "TaskSensor",
    .stack_size = TASK_SENSOR_STACK_SIZE,
    .priority = (osPriority_t)TASK_SENSOR_PRIORITY,
};

const osThreadAttr_t TaskTelemetry_attributes = {
    .name = "TaskTelemetry",
    .stack_size = TASK_TELEM_STACK_SIZE,
    .priority = (osPriority_t)TASK_TELEM_PRIORITY,
};

const osThreadAttr_t TaskRadioRX_attributes = {
    .name = "TaskRadioRX",
    .stack_size = TASK_RADIORX_STACK_SIZE,
    .priority = (osPriority_t)TASK_RADIORX_PRIORITY,
};

const osThreadAttr_t TaskFlightLoop_attributes = {
    .name = "TaskFlightLoop",
    .stack_size = TASK_FLIGHTLOOP_STACK_SIZE,
    .priority = (osPriority_t)TASK_FLIGHTLOOP_PRIORITY,
};

const osThreadAttr_t TaskUARTLogging_attributes = {
    .name = "TaskUARTLogging",
    .stack_size = TASK_LOGGING_STACK_SIZE,
    .priority = (osPriority_t)TASK_LOGGING_PRIORITY,
};

const osThreadAttr_t TaskStartup_attributes = {
    .name = "TaskStartup",
    .stack_size = TASK_STARTUP_STACK_SIZE,
    .priority = (osPriority_t)TASK_STARTUP_PRIORITY,
};

const osThreadAttr_t TaskCommRx_attributes = {
    .name = "TaskCommRx",
    .stack_size = TASK_COMMRX_STACK_SIZE,
    .priority = (osPriority_t)TASK_COMMRX_PRIORITY,
};

QueueHandle_t xLogQueue;
state_t state;
StreamBufferHandle_t crsfStream;
StreamBufferHandle_t commRXStream;
osThreadId_t TaskSensorHandle;
osThreadId_t TaskTelemetryHandle;
osThreadId_t TaskRadioRXHandle;
osThreadId_t TaskFlightLoopHandle;
osThreadId_t TaskUARTLoggingHandle;
osThreadId_t TaskStartupHandle;
osThreadId_t TaskCommRxHandle;

SemaphoreHandle_t imu_mutex;

FC_State fc_state;

static uint8_t crsfStream_Storage[CRSF_BUFFER_SIZE + 1];
static uint8_t commStream_Storage[COMM_BUFFER_SIZE + 1];

static uint8_t logQueue_Storage[LOG_QUEUE_LEN * BUFFER_SIZE];
static StaticStreamBuffer_t crsfStreamStruct;
static StaticStreamBuffer_t commRXStreamStruct;
static StaticQueue_t logQueueStruct;

void Init() {

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim5);

  fc_state.autopilot = MAV_AUTOPILOT_GENERIC;
  fc_state.mode = MAV_MODE_FLAG_TEST_ENABLED;
  if (ENABLE_HIL) {
    fc_state.mode |= MAV_MODE_FLAG_HIL_ENABLED;
  }
  fc_state.type = MAV_TYPE_QUADROTOR;
  fc_state.state = MAV_STATE_BOOT;
  fc_state.system_id = 1;
  fc_state.comp_id = MAV_COMP_ID_AUTOPILOT1;

  crsfStream = xStreamBufferCreateStatic(CRSF_BUFFER_SIZE, 1,
                                         crsfStream_Storage, &crsfStreamStruct);
  commRXStream = xStreamBufferCreateStatic(
      COMM_BUFFER_SIZE, 1, commStream_Storage, &commRXStreamStruct);
  if (crsfStream == NULL || commRXStream == NULL) {
    Error_Handler();
  }

  state.sysid = 1;

  xLogQueue = xQueueCreateStatic(LOG_QUEUE_LEN, BUFFER_SIZE, logQueue_Storage,
                                 &logQueueStruct);
  if (xLogQueue == NULL) {
    Error_Handler();
  }

  imu_mutex = xSemaphoreCreateMutex();

  if (xLogQueue != NULL) {
    TaskUARTLoggingHandle =
        osThreadNew(TaskUARTLogging, NULL, &TaskUARTLogging_attributes);
    if (TaskUARTLoggingHandle == NULL) {
      Error_Handler();
    }
  }
  TaskSensorHandle = osThreadNew(TaskSensor, NULL, &TaskSensor_attributes);
  if (TaskSensorHandle == NULL) {
    Error_Handler();
  }
  TaskCommRxHandle = osThreadNew(TaskCommRx, NULL, &TaskCommRx_attributes);
  if (TaskCommRxHandle == NULL) {
    Error_Handler();
  }
  TaskRadioRXHandle = osThreadNew(TaskRadioRX, NULL, &TaskRadioRX_attributes);
  if (TaskRadioRXHandle == NULL) {
    Error_Handler();
  }
  TaskTelemetryHandle =
      osThreadNew(TaskTelemetry, NULL, &TaskTelemetry_attributes);
  if (TaskTelemetryHandle == NULL) {
    Error_Handler();
  }

  TaskFlightLoopHandle =
      osThreadNew(TaskFlightLoop, NULL, &TaskFlightLoop_attributes);
  if (TaskFlightLoopHandle == NULL) {
    Error_Handler();
  }

  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
  TIM3->CCR1 = TIM3->CNT + 1000;

  uint8_t buf[30] = "All tasks created";
  HAL_UART_Transmit(&huart1, buf, sizeof(buf), HAL_MAX_DELAY);
}
