#include "API.h"
#include "Config.h"
#include "Tasks.h"
#include "common/mavlink.h"
#include "crsf.h"
#include "dshot.h"
#include "logger.h"
#include "main.h"
#include "usart.h"
#include <stdint.h>

#define CRSF_TO_SCALED(x, range, mid)                                          \
  (((x - RC_VAL_MID) * range) / RC_RANGE + mid)

void rc_get_scaled(const crsf_rc_t *raw_rc, rc_scaled_t *rc_scaled) {
  rc_scaled->ts = get_time_since_boot_us();
  rc_scaled->mode = (uint8_t)((raw_rc->ch_data[RC_MAP_CH_MODE] - RC_VAL8_MIN) /
                              (RC_RANGE8 / 2));
  rc_scaled->arm =
      (uint8_t)((raw_rc->ch_data[RC_MAP_CH_ARM] - RC_VAL8_MIN) / (RC_RANGE8));
  switch (rc_scaled->mode) {
  case FLIGHT_MODE_ACRO:
    fc_state.custom_mode = FLIGHT_MODE_ACRO;
    rc_scaled->yaw = CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_YAW],
                                    RC_VEL_RANGE, RC_VEL_MID);
    rc_scaled->roll = CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_ROLL],
                                     RC_VEL_RANGE, RC_VEL_MID);
    rc_scaled->pitch = CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_PITCH],
                                      RC_VEL_RANGE, RC_VEL_MID);
    rc_scaled->throttle =
        CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_THROTTLE], 1, 0.5);
    break;
  case FLIGHT_MODE_POSHOLD:
    // check if mode allowed: need gps
    if (!(fc_state.sensors_enabled & MAV_SYS_STATUS_SENSOR_GPS)) {
      LOG_ERR(0, "NEED_GPS_FOR_POSHOLD");
      break;
    }
    fc_state.custom_mode = FLIGHT_MODE_POSHOLD;
    rc_scaled->yaw = CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_YAW],
                                    RC_VEL_RANGE, RC_VEL_MID);
    rc_scaled->roll = CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_ROLL],
                                     RC_VEL_RANGE, RC_VEL_MID);
    rc_scaled->pitch = CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_PITCH],
                                      RC_VEL_RANGE, RC_VEL_MID);
    rc_scaled->throttle =
        CRSF_TO_SCALED((float)raw_rc->ch_data[RC_MAP_CH_THROTTLE], 1, 0.5);
    break;
  case FLIGHT_MODE_GUIDED:

    // check if mode allowed: need gps
    if (!(fc_state.sensors_enabled & MAV_SYS_STATUS_SENSOR_GPS)) {
      LOG_ERR(0, "NEED_GPS_FOR_GUILDED");
      break;
    }
    fc_state.custom_mode = FLIGHT_MODE_GUIDED;
    break;
  default:
    LOG_ERR(0, "RADIO_MODE_UNSUPPORTED %d", rc_scaled->mode);
    break;
  }
}

static uint8_t radio_rx_buf[RADIORX_DMA_LEN] = {0};
static size_t radio_rx_dma_pos = 0;
static uint32_t notif;
rc_scaled_t rc_scaled;
crsf_rc_t rc_data;

/**
 * @brief Radio RX task
 * @param argument: Not used
 * @retval None
 */
void TaskRadioRX(void *argument) {
  static crsf_frame_t frame;
  static crsf_state_t crsf_state = CRSF_ADDR;
  static uint8_t rx_buf[64] = {0};
  static mavlink_message_t msg;
  static TickType_t cur_tick, last_tick = 0, timeout = pdMS_TO_TICKS(300);
  static size_t n;

  // Begin receiving rc data in circular mode
  HAL_StatusTypeDef res =
      HAL_UART_Receive_DMA(&huart2, radio_rx_buf, RADIORX_DMA_LEN);
  if (res != HAL_OK) {
    Error_Handler();
  }

  for (;;) {
    // Block task until new message is received
    // Task frequency tuned to receiver packet rate (150 Hz)
    n = xStreamBufferReceive(crsfStream, &rx_buf, sizeof(rx_buf),
                             portMAX_DELAY);
    for (size_t i = 0; i < n; i++) {
      if (radio_parse_crsf_byte(&frame, rx_buf[i], &crsf_state)) {
        cur_tick = xTaskGetTickCount();

        xTaskNotifyWait(0, ULONG_MAX, &notif, 0);

        switch (frame.type) {
        case CRSF_TYPE_RC:
          radio_unpack_rc(&rc_data, frame.payload);
          rc_get_scaled(&rc_data, &rc_scaled);

          if ((notif & RADIORX_REQUEST_RAW) &&
              ((cur_tick > last_tick + timeout) || (last_tick == 0))) {
            last_tick = cur_tick;
            notif &= ~RADIORX_REQUEST_RAW;
            mavlink_msg_rc_channels_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, 16,
                rc_data.ch_data[0], rc_data.ch_data[1], rc_data.ch_data[2],
                rc_data.ch_data[3], rc_data.ch_data[4], rc_data.ch_data[5],
                rc_data.ch_data[6], rc_data.ch_data[7], rc_data.ch_data[8],
                rc_data.ch_data[9], rc_data.ch_data[10], rc_data.ch_data[11],
                rc_data.ch_data[12], rc_data.ch_data[13], rc_data.ch_data[14],
                rc_data.ch_data[15], 0, 0, 255);
            comm_tx_send(&msg);
          }
          if ((notif & RADIORX_REQUEST_SCALED) &&
              ((cur_tick > last_tick + timeout) || (last_tick == 0))) {
            last_tick = cur_tick;
            notif &= ~RADIORX_REQUEST_SCALED;
            mavlink_msg_rc_channels_scaled_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, 0, rc_scaled.arm,
                rc_scaled.mode, rc_scaled.roll, rc_scaled.pitch, rc_scaled.yaw,
                rc_scaled.throttle, 0, 0, 255);
            comm_tx_send(&msg);
          }
          break;
        default:
          if (false) {
            last_tick = cur_tick;
            mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                         "RADIORX_MSGTYPE", frame.type, 0, 1,
                                         0);
            comm_tx_send(&msg);
          }
          break;
        }
      }
    }
  }
}

static void radio_dma_to_buffer(size_t npos) {
  size_t n, batch0, rpos;
  rpos = RADIORX_DMA_LEN - radio_rx_dma_pos;

  // handle wrap-around of buffer
  if (npos >= radio_rx_dma_pos) {
    n = npos - radio_rx_dma_pos;
  } else {
    n = rpos + npos; // loop back
  }
  if (n > 0) {
    if (n > rpos) { // would exceed the buffer
      batch0 = rpos;
    } else {
      batch0 = n;
    }
    xStreamBufferSendFromISR(crsfStream, &radio_rx_buf[radio_rx_dma_pos],
                             batch0, NULL);
    if (n > batch0) {
      xStreamBufferSendFromISR(crsfStream, &radio_rx_buf[0], n - batch0, NULL);
    }
    radio_rx_dma_pos = npos;
  }
}

void Radio_UART_RxHalfCpltHandler() {
  radio_dma_to_buffer(RADIORX_DMA_LEN / 2);
}

void Radio_UART_RxCpltHandler() {
  radio_dma_to_buffer(RADIORX_DMA_LEN);
  if (radio_rx_dma_pos == RADIORX_DMA_LEN) {
    radio_rx_dma_pos = 0;
  }
}
