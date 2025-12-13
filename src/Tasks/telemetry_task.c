#include "API.h"
#include "Tasks.h"
#include "common/mavlink.h"
#include "logger.h"
#include "tim.h"
#include "usart.h"

#ifdef FC_ENABLE_RUNTIME_STATS
static char buf[512];
#endif

uint32_t get_time_since_boot_us() {
  return __HAL_TIM_GET_COUNTER(&htim5) * 100;
}

/**
 * @brief Sends telemetry
 * @param argument: Not used
 * @retval None
 */
void TaskTelemetry(void *argument) {
  mavlink_message_t msg;
  uint32_t notif;

  for (;;) {
    mavlink_msg_heartbeat_pack(fc_state.system_id, fc_state.comp_id, &msg,
                               fc_state.type, fc_state.autopilot, fc_state.mode,
                               0, fc_state.state);
    comm_tx_send(&msg);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    xTaskNotifyWait(pdFALSE, 0, &notif, 0);

#ifdef FC_ENABLE_RUNTIME_STATS
    if (notif & TELEM_GET_RUNTIME_STATS) {
      notif &= ~TELEM_GET_RUNTIME_STATS;
      vTaskGetRunTimeStats(buf);
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), portMAX_DELAY);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
