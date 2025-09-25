#include "Tasks.h"
#include "dshot.h"

#define DSHOT_TYPE DSHOT300

/**
 * @brief Flight Loop
 * @param argument: Not used
 * @retval None
 */
void TaskFlightLoop(void *argument) {
  dshot_init(DSHOT_TYPE);
  uint32_t tick = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    tick++;

    // inner PID
    // ESC control
    if (tick % 5 == 0) {
      // outer PID
    }
  }
}