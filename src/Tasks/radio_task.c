#include "Tasks.h"
#include "crsf.h"
#include "usart.h"

#include "logger.h"
#include <stdio.h>

#define DMA_RX_LEN 512

static uint8_t radio_rx_buf[DMA_RX_LEN] = {0};
static size_t radio_rx_dma_pos = 0;

static char RadioRXLog[BUFFER_SIZE] = {0};

/**
 * @brief Radio RX task
 * @param argument: Not used
 * @retval None
 */
void TaskRadioRX(void *argument) {
  crsf_frame_t frame;
  crsf_state_t crsf_state = CRSF_ADDR;
  uint8_t byte;

  uint8_t rx_buf[64];
  crsf_rc_t rc_data;

  // Begin receiving rc data in circular mode
  if (HAL_UART_Receive_DMA(&huart2, radio_rx_buf, DMA_RX_LEN) != HAL_OK) {
    // TODO: handle error
  }

  for (;;) {
    size_t n = xStreamBufferReceive(crsfStream, &rx_buf, sizeof(rx_buf),
                                    pdMS_TO_TICKS(200));
    if (n == 0) {
#if LOG_RADIO_RX
      LOG_WARN("[Radio] No data");
#endif
    } else {
      for (size_t i = 0; i < n; i++) {
        if (radio_parse_crsf_byte(&frame, rx_buf[i], &crsf_state)) {
          // TODO: log and process
          uint8_t len = 2;
          len += snprintf(RadioRXLog + len, sizeof(RadioRXLog) - len,
                          "%d %d %d ", frame.addr, frame.len, frame.type);
          switch (frame.type) {
          case CRSF_TYPE_RC:
            radio_unpack_rc(&rc_data, frame.payload);
            for (size_t i = 0; i < 16; i++) {
              len += snprintf(RadioRXLog + len, sizeof(RadioRXLog) - len, "%d ",
                              rc_data.ch_data[i]);
            }
            break;
          default:
            for (size_t i = 0; i < frame.len - 2; i++) {
              len += snprintf(RadioRXLog + len, sizeof(RadioRXLog) - len, "%d ",
                              frame.payload[i]);
            }
            break;
          }
          // RadioRXLog[0] = len;
          // RadioRXLog[1] = DATA_CTRL;
#if LOG_RADIO_RX
          LOG_INFO(&RadioRXLog);
#endif
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void radio_dma_to_buffer(size_t npos) {
  size_t n, batch0, rpos;
  rpos = DMA_RX_LEN - radio_rx_dma_pos;

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

void Radio_UART_RxHalfCpltHandler() { radio_dma_to_buffer(DMA_RX_LEN / 2); }

void Radio_UART_RxCpltHandler() {
  radio_dma_to_buffer(DMA_RX_LEN);
  if (radio_rx_dma_pos == DMA_RX_LEN) {
    radio_rx_dma_pos = 0;
  }
}
