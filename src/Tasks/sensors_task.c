#include "Tasks.h"
#include "limits.h"
#include "utils.h"

sensor_data_t imu_data;

BMP_CONFIG_PARAMS BMP280_CONFIG_DEFAULT = {.filter_coef = 4,  // x16
                                           .standby_time = 0, // 0.5 ms
                                           .spi3w_en = 0};

BMP_CTRL_MEAS_PARAMS BMP280_CTRL_MEAS_DEFAULT = {
    .mode = BMP_NORMAL,
    .temp_oversampling = 1,     // x1
    .pressure_oversampling = 3, // x4
};

/**
 * @brief Task to handle sensor operations
 * @param argument: Not used
 * @retval None
 */
void TaskSensor(void *argument) {

  I2C_Scan(&hi2c1);

  HAL_StatusTypeDef mpu_status = mpu6050_heartbeat();
  if (mpu_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Accel: no response");
    // TODO: handle error
  }
  if ((mpu6050_set_power_options(CLKSEL_PLLX, 0) != HAL_OK) ||
      (mpu6050_set_config(MPU6050_I2C_BYPASS_EN, MPU6050_DATA_RDY_EN) !=
       HAL_OK)) {
    LOG_CRIT(TASK_SENSOR_ID, "Accel: couldn't configure");
    // TODO: handle error
  } else {
    LOG_INFO(TASK_SENSOR_ID, "Accel: Ready\r\n");
  }

  mpu6050_out mpu6050_data;
  float mpu_temp;
  BMP_CAL_T_PARAMS tp;
  BMP_CAL_P_PARAMS pp;
  BaseType_t queue_status;
  qmc5883_out qmc5883_data;
  float bmp_temp = 0, bmp_pressure = 0;
  BaseType_t xResult;
  uint32_t ulNotifiedValue = 0;
  mavlink_message_t msg;
  TickType_t cur_tick;
  size_t msglen;
  sensor_data_t sdata;

  HAL_StatusTypeDef bmp_status =
      bmp_init(&tp, &pp, BMP280_CTRL_MEAS_DEFAULT, BMP280_CONFIG_DEFAULT);
  if (bmp_status != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Baro: no response");
    // TODO: handle error
  } else {
    LOG_INFO(TASK_SENSOR_ID, "Baro: Ready\r\n");
  }

  // get reference pressure
  bmp_acquire_data(&bmp_pressure, &bmp_temp, tp, pp);
  const float p_ref = bmp_pressure;

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

  // Redirect mag data to mpu6050 for sensor sync
  mpu6050_set_master_ctrl(MPU6050_WAIT_FOR_ES);
  mpu6050_set_config(0, MPU6050_DATA_RDY_EN);
  mpu6050_user_ctrl(MPU6050_I2C_MST_EN);

  if (mpu6050_slv0_init() != HAL_OK) {
    LOG_CRIT(TASK_SENSOR_ID, "Mag: couldn't configure as slave");
  }

  for (;;) {

    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) == pdTRUE) {

      mpu6050_read_data(&mpu6050_data, &qmc5883_data); // DMA
      bmp_acquire_data(&bmp_pressure, &(sdata.bmp_temp), tp,
                       pp); // blocking

      sdata.alt = bmp280_get_altitude(bmp_pressure, p_ref, sdata.bmp_temp);
      // qmc5883_read_data(&qmc5883_data);
      sdata.heading = qmc5883_get_heading(&qmc5883_data, 108.8 / 1000.0);
      mpu_temp = mpu6050_calc_temp(mpu6050_data.temp);
      sdata.accel.accel_x =
          mpu6050_calc_accel(mpu6050_data.accel_x, ACCEL_FS_2G);
      sdata.accel.accel_y =
          mpu6050_calc_accel(mpu6050_data.accel_y, ACCEL_FS_2G);
      sdata.accel.accel_z =
          mpu6050_calc_accel(mpu6050_data.accel_z, ACCEL_FS_2G);
      sdata.gyro.gyro_x = mpu6050_calc_accel(mpu6050_data.gyro_x, FS_SEL_250);
      sdata.gyro.gyro_y = mpu6050_calc_accel(mpu6050_data.gyro_y, FS_SEL_250);
      sdata.gyro.gyro_z = mpu6050_calc_accel(mpu6050_data.gyro_z, FS_SEL_250);

      cur_tick = xTaskGetTickCount();

      if (imu_mutex != NULL) {
        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        imu_data = sdata;
        xSemaphoreGive(imu_mutex);
      }

      msglen = mavlink_msg_highres_imu_pack(
          1, MAV_COMP_ID_AUTOPILOT1, &msg, cur_tick, sdata.accel.accel_x,
          sdata.accel.accel_y, sdata.accel.accel_z, sdata.gyro.gyro_x,
          sdata.gyro.gyro_y, sdata.gyro.gyro_z, qmc5883_data.MagX,
          qmc5883_data.MagY, qmc5883_data.MagZ, bmp_pressure, 0, sdata.alt,
          mpu_temp, 0x0FFF, 0);
      xQueueSend(xLogQueue, &msg, 0);
    }
  }
}