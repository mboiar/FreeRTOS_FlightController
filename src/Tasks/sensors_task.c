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

sensor_data_t imu_data, tmp_data;

GPIO_TypeDef *HCSR04_ECHO_PORT[HCSR04_SENSOR_COUNT] = {GPIOB, GPIOB, GPIOB,
                                                       GPIOB, GPIOB, GPIOA};

uint16_t HCSR04_ECHO_PIN[HCSR04_SENSOR_COUNT] = {
    GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10, GPIO_PIN_13, GPIO_PIN_15, GPIO_PIN_12};

BMP_CONFIG_PARAMS BMP280_CONFIG_DEFAULT = {.filter_coef = 4,  // x16
                                           .standby_time = 0, // 0.5 ms
                                           .spi3w_en = 0};

BMP_CTRL_MEAS_PARAMS BMP280_CTRL_MEAS_DEFAULT = {
    .mode = BMP_NORMAL,
    .temp_oversampling = 1,     // x1
    .pressure_oversampling = 4, // x8
};

float mpu_temp;
BMP_CAL_T_PARAMS tp;
BMP_CAL_P_PARAMS pp;
BaseType_t queue_status;
mavlink_message_t msg;
TickType_t cur_tick, last_tick;
float dt;
size_t msglen;

mag3d_t mag;

float magcal_offset[3];
float magcal_mat[3][3];
float mag_decl = MAG_DECL;
float mag_incl;
gyro3d_t offG;
accel3d_t offA;
float scaleA[3];

static uint32_t notif;
float p_ref; // reference pressure
float bmp_temp, bmp_pressure;
gyro3d_t gyro_offset = {0, 0, 0};
accel3d_t accel_offset = {0, 0, 0};

static eskf_t eskf;

bool distance_sensor_ready_all = false;

TickType_t tick_end, tick_start, last_dist_tick;

float sigma_ww = ESKF_SWW, sigma_wn = ESKF_SWN, sigma_an = ESKF_SAN,
      sigma_aw = ESKF_SAW, sigma_mag = EKSF_SMAG, sigma_baro = EKSF_SBARO;

static float PVcov[15], Qcov[9];

hcsr04_sensor_t dist_sensors[HCSR04_SENSOR_COUNT];
static float dist_buf[HCSR04_SENSOR_COUNT][HCSR04_BUFFER_LEN];
int dist_meas_cnt = 0;
static bool dist_filter_init[HCSR04_SENSOR_COUNT] = {0};

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
    fc_state.state = MAV_STATE_CALIBRATING;
  }

  size_t tick = 0; // 200 Hz

  for (;;) {
    cur_tick = xTaskGetTickCount();
    // BUG: clears all notifications?
    if (xTaskNotifyWait(pdFALSE, 0, &notif, portMAX_DELAY) == pdTRUE) {
      if (notif & SENSOR_CALIBRATION_START) {
        notif &= ~SENSOR_CALIBRATION_START;
        sensors_calibrate();
      } else if (notif & SENSOR_MEASURE) {
        notif &= ~SENSOR_MEASURE;
        tick++;
        if (fc_state.state == MAV_STATE_CALIBRATING) {
          if (tick % 2 == 0) { // 100 Hz
            if (sensors_calibrate_stationary()) {
              // TODO: check if values make sense
              if (mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp,
                                    &gyro_offset, &offA, scaleA) != HAL_OK) {
                // TODO: handle error
              }
              if (qmc5883_read_data(&mag, magcal_offset, magcal_mat) !=
                  HAL_OK) {
                // TODO Handle error
              }
              tmp_data.heading = qmc5883_get_heading(&mag, mag_decl);

              eskf_init(&eskf, sigma_an, sigma_wn, sigma_aw, sigma_ww,
                        &gyro_offset, &mag, &accel_offset);

              // need GPS in case of auto
              if ((fc_state.mode & MAV_MODE_FLAG_GUIDED_ENABLED) ||
                  (fc_state.mode & MAV_MODE_FLAG_AUTO_ENABLED)) {
                if (xTaskNotifyWait(pdFALSE, 0, &notif, portMAX_DELAY) &&
                    (notif & SENSOR_FUSE_GPS)) {
                  fc_state.home_alt = gps_data.alt;
                  fc_state.home_lon = gps_data.lon;
                  fc_state.home_lat = gps_data.lat;
                }
              }
              last_tick = __HAL_TIM_GET_COUNTER(&htim5) * 100; // us
              fc_state.state = MAV_STATE_ACTIVE;
            }
          }

        } else if (fc_state.state == MAV_STATE_ACTIVE) {
          tick_start = xTaskGetTickCount();
          if (mpu6050_read_data(&tmp_data.accel, &tmp_data.gyro, &mpu_temp,
                                &gyro_offset, &offA, scaleA) != HAL_OK) {
            // TODO: handle error
          }
          dt = (__HAL_TIM_GET_COUNTER(&htim5) * 100 - last_tick) /
               1000000.0f; // s
          eskf_predict(&eskf, &tmp_data.accel, &tmp_data.gyro,
                       (float)((dt > 0.0f) ? dt : 1.0f / 1000.0f));
          last_tick = __HAL_TIM_GET_COUNTER(&htim5) * 100;

          tick_end = xTaskGetTickCount();
          msglen = mavlink_msg_param_value_pack(
              1, MAV_COMP_ID_AUTOPILOT1, &msg, " EKF_EXEC_TIME ",
              (float)(tick_end - tick_start), 0, 1, 0);
          // comm_tx_send(&msg);
          if (tick % 10 == 0) { // 20 Hz
            if (bmp_acquire_data(&bmp_pressure, &tmp_data.bmp_temp, tp, pp) !=
                HAL_OK) {
              // TODO Handle error
            }
            tmp_data.alt =
                bmp280_get_altitude(bmp_pressure, p_ref, tmp_data.bmp_temp);
            if (qmc5883_read_data(&mag, magcal_offset, magcal_mat) != HAL_OK) {
              // TODO Handle error
            }
            tmp_data.heading = qmc5883_get_heading(&mag, mag_decl);

            msglen = mavlink_msg_highres_imu_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick,
                tmp_data.accel.accel_x, tmp_data.accel.accel_y,
                tmp_data.accel.accel_z, tmp_data.gyro.gyro_x,
                tmp_data.gyro.gyro_y, tmp_data.gyro.gyro_z, mag.MagX, mag.MagY,
                mag.MagZ, bmp_pressure, bmp_pressure - p_ref, tmp_data.alt,
                tmp_data.bmp_temp, 0xFFFF, 0);
            comm_tx_send(&msg);
            msglen = mavlink_msg_attitude_quaternion_cov_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, eskf.state.quat, 0,
                0, 0, Qcov);
            comm_tx_send(&msg);
            msglen = mavlink_msg_local_position_ned_cov_pack(
                1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick,
                MAV_ESTIMATOR_TYPE_NAIVE, eskf.state.pos[0], eskf.state.pos[1],
                eskf.state.pos[2], eskf.state.vel[0], eskf.state.vel[1],
                eskf.state.vel[2], 0, 0, 0, PVcov);
            comm_tx_send(&msg);

            eskf_update_yaw(&eskf, &mag, sigma_mag);
            eskf_update_baro(&eskf, tmp_data.alt, sigma_baro);
            if (notif & SENSOR_FUSE_GPS) {
              eskf_update_gps(&eskf, &gps_data, fc_state.home_lon,
                              fc_state.home_lat, fc_state.home_alt);
            }
            eskf_get_cov_posvel(&eskf, PVcov);
            eskf_get_cov_orientation(&eskf, Qcov);
          }
          if (tick % 20 == 0) { // 10 Hz
            if (dist_meas_cnt % HCSR04_BUFFER_LEN ==
                HCSR04_BUFFER_LEN - 1) { // filter and report on buffer full
              for (uint8_t i = 0; i < HCSR04_SENSOR_COUNT; i++) {
                dist_sensors[i].last_distance_cm = filter_dist(
                    dist_buf[i], dist_sensors[i].last_distance_cm, 0.8,
                    &dist_filter_init[i], 2, HCSR04_BUFFER_LEN);
                mavlink_msg_distance_sensor_pack(
                    1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, 10, 300,
                    dist_sensors[i].last_distance_cm,
                    MAV_DISTANCE_SENSOR_ULTRASOUND, i,
                    dist_sensors[i].orientation, UINT8_MAX, 0.52f, 0.52f, 0, 0);
              }
              mavlink_msg_command_long_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 2,
                                            MAV_COMP_ID_ONBOARD_COMPUTER, 5000,
                                            0, dist_sensors[0].last_distance_cm,
                                            dist_sensors[1].last_distance_cm,
                                            dist_sensors[2].last_distance_cm,
                                            dist_sensors[3].last_distance_cm,
                                            dist_sensors[4].last_distance_cm,
                                            dist_sensors[5].last_distance_cm,
                                            cur_tick);
              comm_tx_send(&msg);
            }

            dist_meas_cnt++;
            for (uint8_t i = 0; i < HCSR04_SENSOR_COUNT; i++) {
              hcsr04_reset(&dist_sensors[i]);
            }
            hcsr04_trigger();
            last_dist_tick = xTaskGetTickCount();
          }

          // xTaskNotifyWait(pdFALSE, 0, &notif, 0);
          // if (notif & SENSOR_DEBUG_EKF) {
          // notif &= ~SENSOR_DEBUG_EKF;

          // }

          // if (imu_mutex != NULL) {
          //   xSemaphoreTake(imu_mutex, portMAX_DELAY);
          //   imu_data = tmp_data;
          //   xSemaphoreGive(imu_mutex);
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
    if (qmc5883_read_data(&mag, magcal_offset, magcal_mat) != HAL_OK) {
      // TODO handle error
    }

    cur_tick = xTaskGetTickCount();
    msglen = mavlink_msg_highres_imu_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, tmp_data.accel.accel_x,
        tmp_data.accel.accel_y, tmp_data.accel.accel_z, 0, 0, 0, mag.MagX,
        mag.MagY, mag.MagZ, 0, 0, 0, 0, 0xFFFF, 0);
    comm_tx_send(&msg);
    vTaskDelay(pdMS_TO_TICKS(500));
    xTaskNotifyWait(pdFALSE, 0, &notif, 0);
    if (notif & SENSOR_CALIBRATION_STOP) {
      notif &= ~SENSOR_CALIBRATION_STOP;
      break;
    }
  }

  // wait for calibration parameters to arrive
  while (true) {
    xTaskNotifyWait(pdFALSE, 0, &notif, portMAX_DELAY);
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
  offA.accel_x = 0.048761f;
  offA.accel_y = 0.003296f;
  offA.accel_z = -0.06665f;
  scaleA[0] = 1.003368f; // 0.99664f;
  scaleA[1] = 1.012108f; // 0.98804f;
  scaleA[2] = 0.992729f; // 1.00732f;

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

  magcal_offset[0] = -0.219773f;
  magcal_offset[1] = -0.048452f;
  magcal_offset[2] = -0.356323f;

  magcal_mat[0][0] = -0.671190f;
  magcal_mat[0][1] = 0.007412f;
  magcal_mat[0][2] = -0.026167f;
  magcal_mat[1][0] = -0.087054f;
  magcal_mat[1][1] = 0.1715029f;
  magcal_mat[1][2] = -2.1844196f;
  magcal_mat[2][0] = 0.02851924f;
  magcal_mat[2][1] = 3.5780598f;
  magcal_mat[2][2] = 0.28205676f;

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

  for (int i = 0; i < HCSR04_SENSOR_COUNT; i++) {
    hcsr04_init(&dist_sensors[i]);
  }
  hcsr04_trigger();
  last_dist_tick = xTaskGetTickCount();

  return HAL_OK;
}

static uint8_t sensors_calibrate_stationary() {
  static size_t calib_tick;
  calib_tick++;
  if (calib_tick == 100) {
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

void DistanceSensor_RxCpltCallback(uint16_t GPIO_Pin) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int i;
  switch (GPIO_Pin) {
  case GPIO_PIN_1:
    i = 0;
    break;
  case GPIO_PIN_2:
    i = 1;
    break;
  case GPIO_PIN_10:
    i = 2;
    break;
  case GPIO_PIN_12:
    i = 3;
    break;
  case GPIO_PIN_13:
    i = 4;
    break;
  case GPIO_PIN_15:
    i = 5;
    break;
  default:
    i = -1;
    break;
  }
  if (i > -1) {
    GPIO_PinState level =
        HAL_GPIO_ReadPin(HCSR04_ECHO_PORT[i], HCSR04_ECHO_PIN[i]);
    if (level == GPIO_PIN_SET) {
      /* Rising edge */
      dist_sensors[i].t_start_us = __HAL_TIM_GET_COUNTER(&htim5); // 10 kHz
      dist_sensors[i].state = 2; /* WAIT_FALLING */
    } else {
      /* Falling edge */
      if (dist_sensors[i].state == 2 && dist_sensors[i].t_start_us != 0) {
        uint32_t dur =
            __HAL_TIM_GET_COUNTER(&htim5) - dist_sensors[i].t_start_us;
        dist_sensors[i].duration_us = dur;

        // if (dur >= HCSR04_MIN_VALID_US && dur <= HCSR04_MAX_ECHO_US) {
        dist_buf[i][dist_meas_cnt % HCSR04_BUFFER_LEN] =
            duration_to_dist((float)dur * 100.0f, 22.2);
        // } else {
        // dist_buf[i][dist_meas_cnt % HCSR04_BUFFER_LEN] = -1.0f; /* invalid */
        // }
      } else {
        dist_sensors[i].duration_us = 0;
        dist_buf[i][dist_meas_cnt % HCSR04_BUFFER_LEN] = -1.0f;
      }
      dist_sensors[i].state = 0; /* IDLE after measurement */
    }
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}