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
  return __HAL_TIM_GET_COUNTER(&htim5) * 100 * 1;
}

static uint8_t telem_buf[16], len;

/**
 * @brief Sends telemetry
 * @param argument: Not used
 * @retval None
 */
void TaskTelemetry(void *argument) {
  static mavlink_message_t msg;
  static uint32_t notif;

  static uint32_t tick;

  static UBaseType_t watermark[7];

  for (;;) {
    tick++;

    // 10 Hz
    if (tick % 1 == 0) {
      if (fc_state.state == MAV_STATE_ACTIVE) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }

      switch (fc_state.battery_state) {
      case MAV_BATTERY_CHARGE_STATE_UNHEALTHY:
        // battery is faulty: shut down to avoid damage

        LOG_CRIT(0, "BATTERY_UNHEALTHY");
        fc_state.state = MAV_STATE_POWEROFF;
        break;

      case MAV_BATTERY_CHARGE_STATE_EMERGENCY:
        // turn off motors
        LOG_CRIT(0, "BATTERY_EMERGENCY");
        fc_state.state = MAV_STATE_FLIGHT_TERMINATION;
        break;

      case MAV_BATTERY_CHARGE_STATE_CRITICAL:
        // battery is critically low: begin landing
        LOG_ERR(0, "BATTERY_CRITICAL");
        fc_state.custom_mode = FLIGHT_MODE_LAND_UNSUPPORTED;
        break;

      case MAV_BATTERY_CHARGE_STATE_LOW:
        // battery is low: warn
        LOG_WARN(0, "BATTERY_LOW");
        break;

      case MAV_BATTERY_CHARGE_STATE_OK:
        break;

      default:
        break;
      }

      crsf_pack_battery(telem_buf, fc_state.battery_voltage, 0, 0, 0);
      if (HAL_UART_Transmit_IT(&huart2, telem_buf, sizeof(telem_buf)) !=
          HAL_OK) {
        LOG_ERR(0, "UNABLE_TO_SEND_TELEM");
      }

      // Do NOT start another send until previous completed
      xTaskNotifyWait(pdFALSE, 0x01, &notif, pdMS_TO_TICKS(100));

      switch (fc_state.custom_mode) {
      case FLIGHT_MODE_ACRO:
        len = 5;
        crsf_pack_flight_mode(telem_buf, "ACRO", len);
        break;
      case FLIGHT_MODE_STABILIZED:
        len = 5;
        crsf_pack_flight_mode(telem_buf, "STAB", len);
        break;
      case FLIGHT_MODE_POSHOLD:
        len = 8;
        crsf_pack_flight_mode(telem_buf, "POSHOLD", len);
        break;
      case FLIGHT_MODE_GUIDED:
        len = 7;
        crsf_pack_flight_mode(telem_buf, "GUIDED", len);
        break;
      case FLIGHT_MODE_LAND_UNSUPPORTED:
        len = 5;
        crsf_pack_flight_mode(telem_buf, "LAND", len);
        break;
      default:
        len = 8;
        crsf_pack_flight_mode(telem_buf, "UNKNOWN", len);
        break;
      }

      if (HAL_UART_Transmit_IT(&huart2, telem_buf, len + 4) != HAL_OK) {
        LOG_ERR(0, "UNABLE_TO_SEND_TELEM");
      }

      // Do NOT start another send until previous completed
      xTaskNotifyWait(pdFALSE, 0x01, &notif, pdMS_TO_TICKS(100));
    }

    if (tick % 10 == 0) {
      mavlink_msg_heartbeat_pack(fc_state.system_id, fc_state.comp_id, &msg,
                                 fc_state.type, fc_state.autopilot,
                                 fc_state.mode, (uint8_t)fc_state.custom_mode,
                                 fc_state.state);
      comm_tx_send(&msg);

      // if (fc_state.state == MAV_STATE_STANDBY) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      // }

      watermark[0] = uxTaskGetStackHighWaterMark(TaskSensorHandle);
      watermark[1] = uxTaskGetStackHighWaterMark(TaskFlightLoopHandle);
      watermark[2] = uxTaskGetStackHighWaterMark(TaskRadioRXHandle);
      watermark[3] = uxTaskGetStackHighWaterMark(TaskTelemetryHandle);
      watermark[4] = uxTaskGetStackHighWaterMark(TaskUARTLoggingHandle);
      watermark[5] = uxTaskGetStackHighWaterMark(TaskCommRxHandle);
      watermark[6] = uxTaskGetStackHighWaterMark(TaskStartupHandle);
      for (int i = 0; i < 7; i++) {
        if (watermark[i] < 50) {
          LOG_WARN(0, "LOW_STACK %d", i);
        }
      }
    }

#ifdef FC_ENABLE_RUNTIME_STATS
    xTaskNotifyWait(pdFALSE, ULONG_MAX, &notif, 0);
    if (notif & TELEM_GET_RUNTIME_STATS) {
      notif &= ~TELEM_GET_RUNTIME_STATS;
      vTaskGetRunTimeStats(buf);
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), portMAX_DELAY);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Telem_UART_TxCpltHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(TaskTelemetryHandle, 0x01, eSetBits,
                     &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
