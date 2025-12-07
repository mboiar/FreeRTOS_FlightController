#include "API.h"
#include "Tasks.h"
#include "common/mavlink.h"
#include "logger.h"
#include "usart.h"

static char buf[512];

/**
 * @brief Sends telemetry
 * @param argument: Not used
 * @retval None
 */
void TaskTelemetry(void *argument) {
  mavlink_message_t msg;
  uint32_t notif;

  for (;;) {
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_PREFLIGHT, 0, MAV_STATE_CALIBRATING);
    comm_tx_send(&msg);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    xTaskNotifyWait(pdFALSE, 0, &notif, 0);
    if (notif & TELEM_GET_RUNTIME_STATS) {
      notif &= ~TELEM_GET_RUNTIME_STATS;
      vTaskGetRunTimeStats(buf);
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), portMAX_DELAY);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
