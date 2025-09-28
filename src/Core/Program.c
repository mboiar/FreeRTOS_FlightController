#include "Program.h"
#include "cmsis_os.h"

const osThreadAttr_t TaskSensor_attributes = {
    .name = "TaskSensor",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh2,
};

const osThreadAttr_t TaskTelemetry_attributes = {
    .name = "TaskTelemetry",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

const osThreadAttr_t TaskRadioRX_attributes = {
    .name = "TaskRadioRX",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

const osThreadAttr_t TaskFlightLoop_attributes = {
    .name = "TaskFlightLoop",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime2,
};

const osThreadAttr_t TaskUARTLogging_attributes = {
    .name = "TaskUARTLogging",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow1,
};

const osThreadAttr_t TaskStartup_attributes = {
    .name = "TaskStartup",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal1,
};

const size_t LogQueueLen = 5;

QueueHandle_t xLogQueue;
state_t state;
StreamBufferHandle_t crsfStream;
osThreadId_t TaskSensorHandle;
osThreadId_t TaskTelemetryHandle;
osThreadId_t TaskRadioRXHandle;
osThreadId_t TaskFlightLoopHandle;
osThreadId_t TaskUARTLoggingHandle;
osThreadId_t TaskStartupHandle;

void Init() {
  crsfStream = xStreamBufferCreate(256, 20);
  state.sysid = 1;
  xLogQueue = xQueueCreate(LogQueueLen, BUFFER_SIZE);

  TaskSensorHandle = osThreadNew(TaskSensor, NULL, &TaskSensor_attributes);
  TaskRadioRXHandle = osThreadNew(TaskRadioRX, NULL, &TaskRadioRX_attributes);
  TaskTelemetryHandle =
      osThreadNew(TaskTelemetry, NULL, &TaskTelemetry_attributes);

  TaskFlightLoopHandle =
      osThreadNew(TaskFlightLoop, NULL, &TaskFlightLoop_attributes);
  if (xLogQueue != NULL) {
    TaskUARTLoggingHandle =
        osThreadNew(TaskUARTLogging, NULL, &TaskUARTLogging_attributes);
  }
}
