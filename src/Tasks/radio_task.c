#include "API.h"
#include "Tasks.h"
#include "common/mavlink.h"
#include "crsf.h"
#include "main.h"
#include "projdefs.h"
#include "stm32f4xx_hal_def.h"
#include "usart.h"

#include "logger.h"

static uint8_t radio_rx_buf[RADIORX_DMA_LEN] = {0};
static size_t radio_rx_dma_pos = 0;
static uint32_t notif;

/**
 * @brief Radio RX task
 * @param argument: Not used
 * @retval None
 */
void TaskRadioRX(void *argument) {
  crsf_frame_t frame;
  crsf_state_t crsf_state = CRSF_ADDR;
  uint8_t rx_buf[256] = {0};
  crsf_rc_t rc_data;
  mavlink_message_t msg;
  TickType_t cur_tick, last_tick = 0, timeout = pdMS_TO_TICKS(500);
  size_t n;

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

        xTaskNotifyWait(0, RADIORX_DEBUG_RADIO, &notif, 0);

        switch (frame.type) {
        case CRSF_TYPE_RC:
          radio_unpack_rc(&rc_data, frame.payload);

          if ((notif & RADIORX_DEBUG_RADIO) &&
              ((cur_tick > last_tick + timeout) || (last_tick == 0))) {
            last_tick = cur_tick;
            notif &= ~RADIORX_DEBUG_RADIO;
            mavlink_msg_rc_channels_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, 16, frame.payload[0],
                frame.payload[1], frame.payload[2], frame.payload[3],
                frame.payload[4], frame.payload[5], frame.payload[6],
                frame.payload[7], frame.payload[8], frame.payload[9],
                frame.payload[10], frame.payload[11], frame.payload[12],
                frame.payload[13], frame.payload[14], frame.payload[15], 0, 0,
                255);
            comm_tx_send(&msg);
            // vTaskDelay(pdMS_TO_TICKS(300));
          }
          break;
        default:
          if ((notif & RADIORX_DEBUG_RADIO) &&
              (cur_tick > last_tick + timeout)) {
            last_tick = cur_tick;
            notif &= ~RADIORX_DEBUG_RADIO;
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
