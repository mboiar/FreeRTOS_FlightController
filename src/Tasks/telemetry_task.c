#include "Tasks.h"

static char TelemetryLog[BUFFER_SIZE] = {0};

/**
 * @brief Sends telemetry
 * @param argument: Not used
 * @retval None
 */
void TaskTelemetry(void *argument) {
  BaseType_t xQueueStatus;
  BaseType_t xResult;
  mavlink_message_t msg;

  for (;;) {
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_PREFLIGHT, 0, MAV_STATE_CALIBRATING);
    comm_tx_send(&msg);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}