#include "Tasks.h"
#include "dshot.h"

#define DSHOT_TYPE DSHOT300

static uint16_t dshot_data;
static sensor_data_t cur_imu_data;

/**
 * @brief Flight Loop
 * @param argument: Not used
 * @retval None
 */
void TaskFlightLoop(void *argument) {
  dshot_init(DSHOT_TYPE);
  uint32_t tick = 0;

  for (;;) {
    // run with 1 kHz freq
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) == pdTRUE) {
      tick++;

      // get latest sensor data
      if (imu_mutex != NULL) {
        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        cur_imu_data = imu_data;
        xSemaphoreGive(imu_mutex);
      }

      // inner PID TODO

      if (tick % 5 == 0) {
        // outer PID TODO
      }
      dshot_data = 0; // TODO
      dshot_write(dshot_data, 0);
    }
  }
}