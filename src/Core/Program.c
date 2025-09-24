#include "Program.h"


osThreadId_t TaskSensorHandle;
const osThreadAttr_t TaskSensor_attributes = {
  .name = "TaskSensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};

osThreadId_t TaskTelemetryHandle;
const osThreadAttr_t TaskTelemetry_attributes = {
  .name = "TaskTelemetry",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


osThreadId_t TaskRadioRXHandle;
const osThreadAttr_t TaskRadioRX_attributes = {
  .name = "TaskRadioRX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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
  .priority = (osPriority_t) osPriorityNormal1,
};

const size_t LogQueueLen = 5;


static state_t state;

void Init() {
    crsfStream = xStreamBufferCreate(256, 20);
     state.sysid = 1;
xLogQueue = xQueueCreate(LogQueueLen, BUFFER_SIZE);

TaskSensorHandle = osThreadNew(TaskSensor, NULL, &TaskSensor_attributes);
  TaskRadioRXHandle = osThreadNew(TaskRadioRX, NULL, &TaskRadioRX_attributes);
  TaskTelemetryHandle = osThreadNew(TaskTelemetry, NULL, &TaskTelemetry_attributes);

  // TaskFlightLoopHandle = osThreadNew(TaskFlightLoop, NULL, &TaskFlightLoop_attributes);
    if (xLogQueue != NULL) {
        TaskUARTLoggingHandle = osThreadNew(TaskUARTLogging, NULL, &TaskUARTLogging_attributes);
  }

}
