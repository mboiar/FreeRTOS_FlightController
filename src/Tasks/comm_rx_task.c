#include "API.h"
#include "Tasks.h"
#include "cmsis_os.h"
#include "common/mavlink.h"
#include "portmacro.h"
#include "projdefs.h"
#include "usart.h"
#include <stdint.h>

static uint8_t comm_rx_buf[COMMRX_DMA_LEN] = {0};
static size_t comm_rx_dma_pos = 0;

static uint8_t rx_buf[128];

typedef enum {
  MAV_SET_OFFSET_SENSOR_TYPE_GYRO,
  MAV_SET_OFFSET_SENSOR_TYPE_ACC,
  MAV_SET_OFFSET_SENSOR_TYPE_MAG1,
  MAV_SET_OFFSET_SENSOR_TYPE_BARO,
  MAV_SET_OFFSET_SENSOR_TYPE_OF,
  MAV_SET_OFFSET_SENSOR_TYPE_MAG2,
  MAV_SET_OFFSET_SENSOR_TYPE_MAG3,
} MAV_SET_OFFSET_SENSOR_TYPE;

/**
 * @brief Receive messages from serial port
 * @param argument: Not used
 * @retval None
 */
void TaskCommRx(void *argument) {

  mavlink_message_t msg;
  mavlink_status_t status;
  MAV_RESULT mavres;
  size_t n = 0;

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, comm_rx_buf, COMMRX_DMA_LEN) !=
      HAL_OK) {
    // TODO: handle error
  }

  for (;;) {
    mavres = MAV_RESULT_ACCEPTED;
    // BaseType_t res = xStreamBufferIsEmpty(commRXStream);
    n = xStreamBufferReceive(commRXStream, &rx_buf, sizeof(rx_buf),
                             portMAX_DELAY);
    for (size_t i = 0; i < n; i++) {

      if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
          mavlink_command_long_t cmd;
          mavlink_msg_command_long_decode(&msg, &cmd);
          switch (cmd.command) {
          case MAV_CMD_PREFLIGHT_CALIBRATION:
            xTaskNotify(TaskSensorHandle, SENSOR_CALIBRATION_START, eSetBits);
            break;
          case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            switch ((uint8_t)cmd.param1) {
            case MAV_SET_OFFSET_SENSOR_TYPE_MAG1:
              magcal_offset[0] = cmd.param2;
              magcal_offset[1] = cmd.param3;
              magcal_offset[2] = cmd.param4;
              magcal_mat[0][0] = cmd.param5;
              magcal_mat[0][1] = cmd.param6;
              magcal_mat[0][2] = cmd.param7;
              break;
            case MAV_SET_OFFSET_SENSOR_TYPE_MAG2:
              magcal_mat[1][0] = cmd.param2;
              magcal_mat[1][1] = cmd.param3;
              magcal_mat[1][2] = cmd.param4;
              magcal_mat[2][0] = cmd.param5;
              magcal_mat[2][1] = cmd.param6;
              magcal_mat[2][2] = cmd.param7;
              break;
            case MAV_SET_OFFSET_SENSOR_TYPE_MAG3:
              mag_incl = cmd.param2;
              mag_decl = cmd.param3;
              break;
            case MAV_SET_OFFSET_SENSOR_TYPE_ACC:
              offA.accel_x = cmd.param2;
              offA.accel_y = cmd.param3;
              offA.accel_z = cmd.param4;
              scaleA[0] = cmd.param5;
              scaleA[1] = cmd.param6;
              scaleA[2] = cmd.param7;
              xTaskNotify(TaskSensorHandle, SENSOR_CALIBRATION_STOP, eSetBits);
              break;

            default:
              mavres = MAV_RESULT_UNSUPPORTED;
              break;
            }
            break;
          case MAV_CMD_REQUEST_MESSAGE:
            switch ((uint8_t)cmd.param1) {
            case MAV_CMD_REQUEST_RC_RAW:
              xTaskNotify(TaskRadioRXHandle, RADIORX_REQUEST_RAW, eSetBits);
              break;
            case MAV_CMD_REQUEST_RC_SCALED:
              xTaskNotify(TaskRadioRXHandle, RADIORX_REQUEST_SCALED, eSetBits);
              break;
            case MAV_CMD_REQUEST_EKF:
              xTaskNotify(TaskSensorHandle, SENSOR_DEBUG_EKF, eSetBits);
              break;
            case MAV_CMD_REQUEST_RUNTIME_STATS:
              xTaskNotify(TaskTelemetryHandle, TELEM_GET_RUNTIME_STATS,
                          eSetBits);
              break;
            default:
              mavres = MAV_RESULT_UNSUPPORTED;
              break;
            }
            break;
          case MAV_CMD_PREFLIGHT_STORAGE:
            // TODO: actually save to memory
            xTaskNotify(TaskSensorHandle, SENSOR_LOAD_PARAMS, eSetBits);
            break;
          default:
            mavres = MAV_RESULT_UNSUPPORTED;
            break;
          }
          mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                       cmd.command, mavres, 0, 0,
                                       cmd.target_system, cmd.target_component);
          comm_tx_send(&msg);
        }
      }
    }
  }
}

static void commrx_dma_to_buffer(size_t npos) {
  size_t n, batch0, rpos;
  rpos = COMMRX_DMA_LEN - comm_rx_dma_pos;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // handle wrap-around of buffer
  if (npos >= comm_rx_dma_pos) {
    n = npos - comm_rx_dma_pos;
  } else {
    n = rpos + npos; // loop back
  }
  if (n > 0) {
    if (n > rpos) { // would exceed the buffer
      batch0 = rpos;
    } else {
      batch0 = n;
    }
    xStreamBufferSendFromISR(commRXStream, &comm_rx_buf[comm_rx_dma_pos],
                             batch0, &xHigherPriorityTaskWoken);
    if (n > batch0) {
      xStreamBufferSendFromISR(commRXStream, &comm_rx_buf[0], n - batch0,
                               &xHigherPriorityTaskWoken);
    }
    comm_rx_dma_pos = npos;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void CommRx_UART_RxHalfCpltHandler() {
  commrx_dma_to_buffer(COMMRX_DMA_LEN / 2);
  if (comm_rx_dma_pos == COMMRX_DMA_LEN) {
    comm_rx_dma_pos = 0;
  }
}

void CommRx_UART_RxCpltHandler() {
  commrx_dma_to_buffer(COMMRX_DMA_LEN);
  if (comm_rx_dma_pos == COMMRX_DMA_LEN) {
    comm_rx_dma_pos = 0;
  }
}

void CommRx_UARTEx_RxEventHandler(uint16_t Size) {
  commrx_dma_to_buffer(Size);
  if (comm_rx_dma_pos == COMMRX_DMA_LEN) {
    comm_rx_dma_pos = 0;
  }
}
