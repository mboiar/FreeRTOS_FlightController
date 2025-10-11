#include "Tasks.h"
#include "utils.h"

typedef enum { INIT, CALIBRATING, READY } SENSORS_STATE;

static void sensors_calibrate();
static uint8_t sensors_calibrate_stationary();
static void sensors_init();

sensor_data_t imu_data, tmp_data;
SENSORS_STATE sstate;

BMP_CONFIG_PARAMS BMP280_CONFIG_DEFAULT = {.filter_coef = 4,  // x16
                                           .standby_time = 0, // 0.5 ms
                                           .spi3w_en = 0};

BMP_CTRL_MEAS_PARAMS BMP280_CTRL_MEAS_DEFAULT = {
    .mode = BMP_NORMAL,
    .temp_oversampling = 1,     // x1
    .pressure_oversampling = 3, // x4
};

float mpu_temp;
BMP_CAL_T_PARAMS tp;
BMP_CAL_P_PARAMS pp;
BaseType_t queue_status;
mavlink_message_t msg;
TickType_t cur_tick, last_tick, dt;
size_t msglen;

mag3d_t mag;

float magcal_offset[3];
float magcal_mat[3][3];
float mag_decl;
float mag_incl;
gyro3d_t offG;
accel3d_t offA;
float scaleA[3];

static uint32_t notif;
float p_ref; // reference pressure
float bmp_temp, bmp_pressure;
gyro3d_t gyro_offset;

static eskf_t eskf;

float sigma_ww = 0, sigma_wn = 0, sigma_an = 0, sigma_aw = 0, sigma_mag = 0,
      sigma_baro = 0;

/**
 * @brief Task to handle sensor operations
 * @param argument: Not used
 * @retval None
 */
void TaskSensor(void *argument) {
  sstate = INIT;
  sensors_init();

  sstate = CALIBRATING;

  size_t tick = 0;

  for (;;) {
    cur_tick = xTaskGetTickCount();
    // BUG: clears all notifications?
    if (xTaskNotifyWait(pdFALSE, pdTRUE, &notif, portMAX_DELAY) == pdTRUE) {
      if (notif & SENSOR_CALIBRATION_START) {
        sensors_calibrate();
      } else if (notif & SENSOR_MEASURE) {
        tick++;
        if (sstate == CALIBRATING) {
          if (tick % 10 == 0) { // 100 Hz
            if (sensors_calibrate_stationary()) {
              // TODO: check if values make sense
              eskf_init(&eskf, sigma_an, sigma_wn, sigma_aw, sigma_ww);
              sstate = READY;
            }
          }

        } else if (sstate == READY) {

          mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp, &offG,
                            &offA, scaleA);
          eskf_predict(&eskf, &tmp_data.accel, &tmp_data.gyro,
                       (float)(last_tick - cur_tick));
          if (tick % 10 == 0) { // 100 Hz
            bmp_acquire_data(&bmp_pressure, &tmp_data.bmp_temp, tp,
                             pp); // blocking
            tmp_data.alt =
                bmp280_get_altitude(bmp_pressure, p_ref, tmp_data.bmp_temp);
            qmc5883_read_data(&mag, magcal_offset, magcal_mat);
            tmp_data.heading = qmc5883_get_heading(&mag, mag_decl);
            // eskf_update_yaw(&eskf, tmp_data.heading, sigma_mag);
            // eskf_update_alt(&eskf, tmp_data.alt, sigma_baro);
            msglen = mavlink_msg_attitude_quaternion_cov_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, eskf.state.quat, 0,
                0, 0, eskf.P);
            msglen = mavlink_msg_local_position_ned_cov_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick,
                MAV_ESTIMATOR_TYPE_NAIVE, eskf.state.pos[0], eskf.state.pos[1],
                eskf.state.pos[2], eskf.state.vel[0], eskf.state.vel[1],
                eskf.state.vel[2], 0, 0, 0, eskf.P);
          }

          if (imu_mutex != NULL) {
            xSemaphoreTake(imu_mutex, portMAX_DELAY);
            imu_data = tmp_data;
            xSemaphoreGive(imu_mutex);
          }
        }
      }
    }
    last_tick = cur_tick;
  }
}

static void sensors_calibrate() {
  // TODO: signal start of calibration

  // collect calibration data
  while (true) {
    mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp, &offG, &offA,
                      scaleA); // DMA
    qmc5883_read_data(&mag, magcal_offset, magcal_mat);

    cur_tick = xTaskGetTickCount();
    msglen = mavlink_msg_highres_imu_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, tmp_data.accel.accel_x,
        tmp_data.accel.accel_y, tmp_data.accel.accel_z, 0, 0, 0, mag.MagX,
        mag.MagY, mag.MagZ, 0, 0, 0, 0, 0xFFFF, 0);
    comm_tx_send(&msg);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskNotifyWait(pdFALSE, pdTRUE, &notif, 0);
    if (notif & SENSOR_CALIBRATION_STOP) {
      break;
    }
  }

  // wait for calibration parameters to arrive
  while (true) {
    xTaskNotifyWait(pdFALSE, pdTRUE, &notif, portMAX_DELAY);
    if (notif & SENSOR_LOAD_PARAMS) {
      // TODO: Load into non-volatile memory
      break;
    }
  }
  // TODO: signal end of calibration
}

static void sensors_init() {
  I2C_Scan(&hi2c1);

  HAL_StatusTypeDef mpu_status = mpu6050_heartbeat();
  if (mpu_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Accel: no response");
    // TODO: handle error
  }
  mpu6050_user_ctrl(0);
  if ((mpu6050_set_power_options(CLKSEL_PLLX, 0) != HAL_OK) ||
      (mpu6050_set_config(MPU6050_I2C_BYPASS_EN, MPU6050_DATA_RDY_EN,
                          SMPRT_DIV) != HAL_OK)) {
    LOG_CRIT(TASK_SENSOR_ID, "Accel: couldn't configure");
    // TODO: handle error
  } else {
    LOG_INFO(TASK_SENSOR_ID, "Accel: Ready\r\n");
  }

  bmp_temp = 0;
  bmp_pressure = 0;
  p_ref = 0;
  offG.gyro_x = 0;
  offG.gyro_y = 0;
  offG.gyro_z = 0;
  offA.accel_x = 0;
  offA.accel_y = 0;
  offA.accel_z = 0;
  for (int i = 0; i < 3; i++) {
    scaleA[i] = 1;
  }

  // initialize mag offsets
  for (size_t i = 0; i < 3; i++) {
    magcal_offset[i] = 0;
    for (size_t j = 0; j < 3; j++) {
      magcal_mat[i][j] = 0;
      if (i == j) {
        magcal_mat[i][j] = 1;
      }
    }
  }

  HAL_StatusTypeDef bmp_status =
      bmp_init(&tp, &pp, BMP280_CTRL_MEAS_DEFAULT, BMP280_CONFIG_DEFAULT);
  if (bmp_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Baro: no response");
    // TODO: handle error
  } else {
    LOG_INFO(TASK_SENSOR_ID, "Baro: Ready\r\n");
  }

  // Configure QMC5883 magnetometer
  HAL_StatusTypeDef qmc_status = qmc5883_heartbeat();
  if (qmc_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Mag: no response");
    // TODO: handle error
  }
  if ((qmc5883_set_config(QMC5883_CONTINUOUS | ODR_100HZ | RNG_2G | OSR_512) !=
       HAL_OK) ||
      (qmc5883_set_ctrl(INT_DISABLE | ROL_PNT_NORMAL) != HAL_OK)) {
    LOG_CRIT(TASK_SENSOR_ID, "Mag: couldn't configure");
    // TODO: handle error
  } else {
    LOG_INFO(TASK_SENSOR_ID, "Mag: Ready\r\n");
  }
}

static uint8_t sensors_calibrate_stationary() {
  static size_t calib_tick;
  calib_tick++;
  if (calib_tick == 300) {
    return 1;
  }
  mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp, &offG, &offA,
                    scaleA);
  bmp_acquire_data(&bmp_pressure, &(tmp_data.bmp_temp), tp,
                   pp); // blocking
  p_ref =
      p_ref * (calib_tick - 1) / calib_tick + tmp_data.bmp_temp / calib_tick;
  gyro_offset.gyro_x = gyro_offset.gyro_x * (calib_tick - 1) / calib_tick +
                       tmp_data.gyro.gyro_x / calib_tick;
  gyro_offset.gyro_y = gyro_offset.gyro_y * (calib_tick - 1) / calib_tick +
                       tmp_data.gyro.gyro_y / calib_tick;
  gyro_offset.gyro_z = gyro_offset.gyro_z * (calib_tick - 1) / calib_tick +
                       tmp_data.gyro.gyro_z / calib_tick;
  return 0;
}