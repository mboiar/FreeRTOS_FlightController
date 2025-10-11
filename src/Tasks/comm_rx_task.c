#include "Tasks.h"
#include "usart.h"

// osThreadId_t TaskUARTLoggingHandle;

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
  uint32_t ulNotifiedValue;
  HAL_StatusTypeDef UARTStatus;
  BaseType_t xQueueStatus;
  BaseType_t xResult;
  // uint8_t data_buf[BUFFER_SIZE] = {0};
  uint8_t data_buf = 0;
  size_t length;
  // TickType_t timestamp;
  mavlink_message_t msg, resp;
  mavlink_status_t status;
  MAV_RESULT mavres;
  for (;;) {
    mavres = MAV_RESULT_ACCEPTED;
    UARTStatus = HAL_UART_Receive_DMA(&huart1, &data_buf, sizeof(data_buf));
    xResult =
        xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
    if (mavlink_parse_char(MAVLINK_COMM_0, data_buf, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);
        if (cmd.command == MAV_CMD_PREFLIGHT_CALIBRATION && cmd.param2 == 1) {
          xTaskNotify(TaskSensorHandle, SENSOR_CALIBRATION_START, eSetBits);
        }
        if (cmd.command == MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS) {
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
        }
        if (cmd.command == MAV_CMD_PREFLIGHT_STORAGE) {
          // TODO: actually save to memory
          xTaskNotify(TaskSensorHandle, SENSOR_LOAD_PARAMS, eSetBits);
        }
        mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, &resp,
                                     cmd.command, mavres, 0, 0,
                                     cmd.target_system, cmd.target_component);
        comm_tx_send(&resp);
      }
    }
  }
}

void CommRx_UART_RxCpltHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(TaskCommRxHandle, 0x01, eSetBits,
                     &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
