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
  static mavlink_message_t msg;
  static uint32_t notif;

  static uint32_t tick;

  static UBaseType_t watermark[5];

  for (;;) {
    tick++;
    if (tick % 1 == 0) {
      if (fc_state.state == MAV_STATE_ACTIVE) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }
    }
    if (tick % 10 == 0) {
      mavlink_msg_heartbeat_pack(fc_state.system_id, fc_state.comp_id, &msg,
                                 fc_state.type, fc_state.autopilot,
                                 fc_state.mode, fc_state.custom_mode,
                                 fc_state.state);
      comm_tx_send(&msg);

      if (fc_state.state == MAV_STATE_STANDBY) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }

      watermark[0] = uxTaskGetStackHighWaterMark(TaskSensorHandle);
      watermark[1] = uxTaskGetStackHighWaterMark(TaskFlightLoopHandle);
      watermark[2] = uxTaskGetStackHighWaterMark(TaskRadioRXHandle);
      watermark[3] = uxTaskGetStackHighWaterMark(TaskTelemetryHandle);
      watermark[4] = uxTaskGetStackHighWaterMark(TaskUARTLoggingHandle);
      for (int i = 0; i < 5; i++) {
        if (watermark[i] < 50) {
          LOG_WARN(0, "LOW_STACK %d", i);
        }
      }
    }

#ifdef FC_ENABLE_RUNTIME_STATS
    xTaskNotifyWait(pdFALSE, 0, &notif, 0);
    if (notif & TELEM_GET_RUNTIME_STATS) {
      notif &= ~TELEM_GET_RUNTIME_STATS;
      vTaskGetRunTimeStats(buf);
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), portMAX_DELAY);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
