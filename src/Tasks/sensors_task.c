#include "API.h"
#include "Tasks.h"
#include "common/mavlink.h"
#include "hcsr04.h"
#include "imu.h"
#include "logger.h"
#include "portmacro.h"
#include "projdefs.h"
#include "tim.h"
#include "utils.h"

static void sensors_calibrate();
static uint8_t sensors_calibrate_stationary();
static HAL_StatusTypeDef sensors_init();

sensor_data_t imu_data;
static sensor_data_t tmp_data;

const BMP_CONFIG_PARAMS BMP280_CONFIG_DEFAULT = {.filter_coef = 4,  // x16
                                                 .standby_time = 0, // 0.5 ms
                                                 .spi3w_en = 0};

const BMP_CTRL_MEAS_PARAMS BMP280_CTRL_MEAS_DEFAULT = {
    .mode = BMP_NORMAL,
    .temp_oversampling = 2,    // 1,     // x1
    .pressure_oversampling = 5 // 4, // x8
};

static float mpu_temp;
static BMP_CAL_T_PARAMS tp;
static BMP_CAL_P_PARAMS pp;
static BaseType_t queue_status;
static mavlink_message_t msg;
static TickType_t cur_tick, last_tick;
static float dt;
static size_t msglen;

static mag3d_t mag;

float magcal_offset[3];
float magcal_mat[3][3];
float mag_decl = MAG_DECL;
float mag_incl;
static gyro3d_t offG;
accel3d_t offA;
float scaleA[3];

static uint32_t notif;
float p_ref; // reference pressure
float bmp_temp, bmp_pressure;
static gyro3d_t gyro_offset = {0, 0, 0};
static accel3d_t accel_offset = {0, 0, 0};

eskf_t eskf;

static TickType_t tick_end, tick_start;

static float sigma_ww = ESKF_SWW, sigma_wn = ESKF_SWN, sigma_an = ESKF_SAN,
             sigma_aw = ESKF_SAW, sigma_mag = EKSF_SMAG,
             sigma_baro = EKSF_SBARO;

static float PVcov[21], Qcov[9];

static bool home_set;
static float pm[5];

/**
 * @brief Task to handle sensor operations
 * @param argument: Not used
 * @retval None
 */
void TaskSensor(void *argument) {
  if (sensors_init() != HAL_OK) {
    mavlink_msg_statustext_pack(fc_state.system_id, fc_state.comp_id, &msg,
                                MAV_SEVERITY_CRITICAL, "Sensor init error", 0,
                                0);
    comm_tx_send(&msg);
    vTaskSuspendAll();
  }
  if (fc_state.state == MAV_STATE_BOOT) {
    fc_state.state = MAV_STATE_STANDBY;
  }

  size_t tick = 0; // 200 Hz

  for (;;) {
    if (xTaskNotifyWait(pdFALSE, ULONG_MAX, &notif, portMAX_DELAY) == pdTRUE) {
      cur_tick = xTaskGetTickCount();

      if (notif & SENSOR_CALIBRATION_START) {
        notif &= ~SENSOR_CALIBRATION_START;
        sensors_calibrate();
      } else if (notif & SENSOR_MEASURE) {
        notif &= ~SENSOR_MEASURE;
        tick++;
        if (fc_state.state == MAV_STATE_CALIBRATING) {
          if (notif & SENSOR_FUSE_GPS) {
            notif &= ~SENSOR_FUSE_GPS;
            // TODO set health

            fc_state.home_lat = gps_data.lat;
            fc_state.home_lon = gps_data.lon;
            fc_state.home_alt = -gps_data.alt;
            home_set = true;
          }

          if (tick % 2 == 0) { // 100 Hz

            if (sensors_calibrate_stationary()) {
              // TODO: check if values make sense
              if (mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp,
                                    &gyro_offset, &offA, scaleA) != HAL_OK) {
                // TODO: handle error
              }
              if (qmc5883_read_data(&mag) != HAL_OK) {
                // TODO Handle error
              }
              // tmp_data.heading = qmc5883_get_heading(&mag, mag_decl);

              eskf_init(&eskf, sigma_an, sigma_wn, sigma_aw, sigma_ww,
                        &gyro_offset, &mag, &accel_offset, magcal_offset,
                        magcal_mat);

              // need GPS in case of auto
              // if ((fc_state.mode & MAV_MODE_FLAG_GUIDED_ENABLED) ||
              //     (fc_state.mode & MAV_MODE_FLAG_AUTO_ENABLED)) {
              //   do {
              //     xTaskNotifyWait(pdFALSE, 0, &notif, portMAX_DELAY);
              //     fc_state.home_alt = gps_data.alt;
              //     fc_state.home_lon = gps_data.lon;
              //     fc_state.home_lat = gps_data.lat;
              //   } while (!(notif & SENSOR_FUSE_GPS));
              // }
              last_tick = __HAL_TIM_GET_COUNTER(&htim5) * 100; // us
              xTaskNotify(TaskFlightLoopHandle, PID_ARM_READY, eSetBits);
            }
          }

        } else if ((fc_state.state == MAV_STATE_ACTIVE)) {
          tick_start = xTaskGetTickCount();
          if (mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp,
                                &gyro_offset, &offA, scaleA) != HAL_OK) {
            fc_state.state = MAV_STATE_EMERGENCY;
          }
          dt = (__HAL_TIM_GET_COUNTER(&htim5) * 100 - last_tick) /
               1000000.0f; // s

          if (eskf_predict(&eskf, &tmp_data.accel, &tmp_data.gyro, dt) < 0) {
            fc_state.state = MAV_STATE_CRITICAL;
          }

          last_tick = __HAL_TIM_GET_COUNTER(&htim5) * 100;

          tick_end = xTaskGetTickCount();

          if (notif & SENSOR_FUSE_GPS) {
            notif &= ~SENSOR_FUSE_GPS;
            // TODO set health

            LOG_INFO(0, "GPS_UPDATE");

            if (eskf_update_gps(&eskf, &gps_data, fc_state.home_lon,
                                fc_state.home_lat, fc_state.home_alt, 0.1, 0.1,
                                0.1, pm) != 0) {
              LOG_ERR(0, "INVALID_GPS_UPDATE");
            }
          }

          // msglen = mavlink_msg_param_value_pack(
          //     1, MAV_COMP_ID_AUTOPILOT1, &msg, " EKF_EXEC_TIME ",
          //     (float)(tick_end - tick_start), 0, 1, 0);
          // comm_tx_send(&msg);
          if (tick % 10 == 0) { // 20 Hz
            if (bmp_acquire_data(&bmp_pressure, &tmp_data.bmp_temp, tp, pp) !=
                HAL_OK) {
              // TODO Handle error
            }
            tmp_data.alt =
                bmp280_get_altitude(bmp_pressure, p_ref, tmp_data.bmp_temp);
            if (qmc5883_read_data(&mag) != HAL_OK) {
              // TODO Handle error
            }
            // tmp_data.heading = qmc5883_get_heading(&mag, mag_decl);

            if (eskf_update_yaw(&eskf, &mag, sigma_mag, &tmp_data.heading,
                                magcal_offset, magcal_mat) < 0 ||
                eskf_update_baro(&eskf, tmp_data.alt, sigma_baro) < 0) {
              // TODO disable auto, revert to manual rc
            }

            msglen = mavlink_msg_highres_imu_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick,
                tmp_data.accel.accel_x, tmp_data.accel.accel_y,
                tmp_data.accel.accel_z, tmp_data.gyro.gyro_x,
                tmp_data.gyro.gyro_y, tmp_data.gyro.gyro_z, mag.MagX, mag.MagY,
                mag.MagZ, tmp_data.heading, pm[4], tmp_data.alt, pm[5], 0xFFFF,
                0);
            comm_tx_send(&msg);
          }
          if (tick % 20 == 0) {
            eskf_get_cov_posvel(&eskf, PVcov);
            eskf_get_cov_orientation(&eskf, Qcov);
            msglen = mavlink_msg_attitude_quaternion_cov_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, eskf.state.quat, 0,
                0, 0, Qcov);
            comm_tx_send(&msg);
            msglen = mavlink_msg_local_position_ned_cov_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick,
                MAV_ESTIMATOR_TYPE_AUTOPILOT, eskf.state.pos[0],
                eskf.state.pos[1], eskf.state.pos[2], eskf.state.vel[0],
                eskf.state.vel[1], eskf.state.vel[2], acc_glob[0], acc_glob[1],
                acc_glob[2], PVcov);
            comm_tx_send(&msg);
          }

          // xTaskNotifyWait(pdFALSE, 0, &notif, 0);
          // if (notif & SENSOR_DEBUG_EKF) {
          // notif &= ~SENSOR_DEBUG_EKF;

          // }

          // if (imu_mutex != NULL) {
          //   xSemaphoreTake(imu_mutex, portMAX_DELAY);
          imu_data = tmp_data;
          //   xSemaphoreGive(imu_mutex);
        } else if (fc_state.state == MAV_STATE_CRITICAL) {
          LOG_CRIT(0, "FC_STATE_CRITICAL");
        }
      }
    }
  }
}

static void sensors_calibrate() {
  // TODO: signal start of calibration

  // collect calibration data
  while (true) {
    if (mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp, &offG,
                          &offA, scaleA) != HAL_OK) {
      // TODO handle error
    }
    // if (qmc5883_read_data(&mag, magcal_offset, magcal_mat) != HAL_OK) {
    //   // TODO handle error
    // }

    cur_tick = xTaskGetTickCount();
    msglen = mavlink_msg_highres_imu_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, tmp_data.accel.accel_x,
        tmp_data.accel.accel_y, tmp_data.accel.accel_z, 0, 0, 0, mag.MagX,
        mag.MagY, mag.MagZ, 0, 0, 0, 0, 0xFFFF, 0);
    comm_tx_send(&msg);
    vTaskDelay(pdMS_TO_TICKS(500));
    // xTaskNotifyWait(pdFALSE, SENSOR_CALIBRATION_STOP, &notif, 0);
    if (notif & SENSOR_CALIBRATION_STOP) {
      notif &= ~SENSOR_CALIBRATION_STOP;
      break;
    }
  }

  // wait for calibration parameters to arrive
  while (true) {
    // xTaskNotifyWait(pdFALSE, SENSOR_LOAD_PARAMS, &notif, portMAX_DELAY);
    if (notif & SENSOR_LOAD_PARAMS) {
      notif &= ~SENSOR_LOAD_PARAMS;
      // TODO: Load into non-volatile memory
      break;
    }
  }
  // TODO: signal end of calibration
}

static HAL_StatusTypeDef sensors_init() {
  // vTaskDelay(pdMS_TO_TICKS(1000));
  I2C_Scan(&hi2c1);

  HAL_StatusTypeDef mpu_status = mpu6050_heartbeat();
  if (mpu_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Accel: no response");
    // TODO: handle error
    return HAL_ERROR;
  }
  mpu6050_user_ctrl(0);
  if ((mpu6050_set_power_options(CLKSEL_PLLX, 0) != HAL_OK) ||
      (mpu6050_set_config(MPU6050_I2C_BYPASS_EN, MPU6050_DATA_RDY_EN, 0, 2) !=
       HAL_OK) ||
      mpu6050_set_gyro_accel_config(FS_SEL_250, AFS_2G)) {
    LOG_CRIT(TASK_SENSOR_ID, "Accel: couldn't configure");
    // TODO: handle error
    return HAL_ERROR;

  } else {
    LOG_INFO(TASK_SENSOR_ID, "Accel: Ready\r\n");
  }

  bmp_temp = 0;
  bmp_pressure = 0;
  p_ref = 0;
  offG.gyro_x = 0;
  offG.gyro_y = 0;
  offG.gyro_z = 0;
  offA.accel_y = -0.00505; // 0.048761f;
  offA.accel_x = 0.04695;  // 0.003296f;
  offA.accel_z = -0.0774;  //-0.06665f;
  scaleA[1] = 1.00306;     // 1.003368f; // 0.99664f;
  scaleA[0] = 1.00306;     // 1.012108f; // 0.98804f;
  scaleA[2] = 0.98629;     // 0.992729f; // 1.00732f;

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

  magcal_offset[0] = -0.0463f;
  magcal_offset[1] = -0.17096f;
  magcal_offset[2] = -0.07806f;

  magcal_mat[0][0] = 0.98162718f;
  magcal_mat[0][1] = -0.01994497f;
  magcal_mat[0][2] = -0.18976372f;
  magcal_mat[1][0] = 0.0f;
  magcal_mat[1][1] = 0.9945219f;
  magcal_mat[1][2] = -0.10452846f;
  magcal_mat[2][0] = 0.190809f;
  magcal_mat[2][1] = 0.10260798f;
  magcal_mat[2][2] = 0.97624973f;

  HAL_StatusTypeDef bmp_status;
  bmp_status =
      bmp_init(&tp, &pp, BMP280_CTRL_MEAS_DEFAULT, BMP280_CONFIG_DEFAULT);
  if (bmp_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Baro: no response");
    // TODO: handle error
    return HAL_ERROR;

  } else {
    LOG_INFO(TASK_SENSOR_ID, "Baro: Ready\r\n");
  }

  // Configure QMC5883 magnetometer
  HAL_StatusTypeDef qmc_status = qmc5883_heartbeat();
  if (qmc_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Mag: no response");
    // TODO: handle error
    return HAL_ERROR;
  }
  if ((qmc5883_set_config(QMC5883_CONTINUOUS | ODR_100HZ | RNG_2G | OSR_512) !=
       HAL_OK) ||
      (qmc5883_set_ctrl(INT_DISABLE | ROL_PNT_NORMAL) != HAL_OK)) {
    LOG_CRIT(TASK_SENSOR_ID, "Mag: couldn't configure");
    // TODO: handle error
    return HAL_ERROR;
  } else {
    LOG_INFO(TASK_SENSOR_ID, "Mag: Ready\r\n");
  }

  return HAL_OK;
}

static uint8_t sensors_calibrate_stationary() {
  static size_t calib_tick;
  calib_tick++;
  if (calib_tick % 100 == 0 && home_set) {
    LOG_INFO(0, "BIAS %.4f %.4f %.4f %.4f", gyro_offset.gyro_x,
             gyro_offset.gyro_y, gyro_offset.gyro_z, p_ref);
    home_set = false;
    return 1;
  }
  mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp, &offG, &offA,
                    scaleA);
  bmp_acquire_data(&bmp_pressure, &(tmp_data.bmp_temp), tp,
                   pp); // blocking
  p_ref = p_ref * (calib_tick - 1) / calib_tick + bmp_pressure / calib_tick;
  gyro_offset.gyro_x = gyro_offset.gyro_x * (calib_tick - 1) / calib_tick +
                       tmp_data.gyro.gyro_x / calib_tick;
  gyro_offset.gyro_y = gyro_offset.gyro_y * (calib_tick - 1) / calib_tick +
                       tmp_data.gyro.gyro_y / calib_tick;
  gyro_offset.gyro_z = gyro_offset.gyro_z * (calib_tick - 1) / calib_tick +
                       tmp_data.gyro.gyro_z / calib_tick;
  accel_offset.accel_x = accel_offset.accel_x * (calib_tick - 1) / calib_tick +
                         tmp_data.accel.accel_x / calib_tick;
  accel_offset.accel_y = accel_offset.accel_y * (calib_tick - 1) / calib_tick +
                         tmp_data.accel.accel_y / calib_tick;
  accel_offset.accel_z = accel_offset.accel_z * (calib_tick - 1) / calib_tick +
                         tmp_data.accel.accel_z / calib_tick;

  return 0;
}
