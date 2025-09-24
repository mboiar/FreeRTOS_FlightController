#include "Tasks.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "qmc5883.h"
#include "utils.h"

typedef struct {
  accel_3d accel;
  gyro_3d gyro;
  float heading;
  float alt;
  // TickType_t timestamp;
  float bmp_temp;
} sensor_data_t;

static uint8_t SensorDataBuffer[BUFFER_SIZE] = {0};

/**
 * @brief Task to handle sensor operations
 * @param argument: Not used
 * @retval None
 */
void TaskSensor(void *argument) {

  I2C_Scan(&hi2c1);

  HAL_StatusTypeDef mpu_status = mpu6050_heartbeat();
  mpu6050_set_power_options(CLKSEL_PLLX, 0);
  mpu6050_set_config(MPU6050_I2C_BYPASS_EN, MPU6050_DATA_RDY_EN);

  mpu6050_out mpu6050_data;
  float mpu_temp;
  BMP_CAL_T_PARAMS tp;
  BMP_CAL_P_PARAMS pp;
  BaseType_t queue_status;

  char SensorLog[BUFFER_SIZE] = {0};

  BMP_CONFIG_PARAMS BMP280_CONFIG_DEFAULT = {.filter_coef = 4,  // x16
                                             .standby_time = 0, // 0.5 ms
                                             .spi3w_en = 0};

  BMP_CTRL_MEAS_PARAMS BMP280_CTRL_MEAS_DEFAULT = {
      .mode = BMP_NORMAL,
      .temp_oversampling = 1,     // x1
      .pressure_oversampling = 3, // x4
  };
  HAL_StatusTypeDef bmp_status =
      bmp_init(&tp, &pp, BMP280_CTRL_MEAS_DEFAULT, BMP280_CONFIG_DEFAULT);
  float bmp_temp = 0, bmp_pressure = 0;

  // get reference pressure
  bmp_acquire_data(&bmp_pressure, &bmp_temp, tp, pp);
  const float p_ref = bmp_pressure;

  // Configure QMC5883 magnetometer
  HAL_StatusTypeDef qmc_status = qmc5883_heartbeat();
  qmc5883_set_config(QMC5883_CONTINUOUS | ODR_100HZ | RNG_2G | OSR_512);
  qmc5883_set_ctrl(INT_DISABLE | ROL_PNT_NORMAL);
  qmc5883_out qmc5883_data;

  // Redirect mag data to mpu6050 for sensor sync
  mpu6050_set_master_ctrl(MPU6050_WAIT_FOR_ES);
  mpu6050_set_config(0, MPU6050_DATA_RDY_EN);
  mpu6050_user_ctrl(MPU6050_I2C_MST_EN); // | MPU6050_DMP_EN | MPU6050_FIFO_EN);

  qmc_status = mpu6050_slv0_init();

  sensor_data_t sdata;

  // mpu_status = mpu6050_dmp_load_firmware();

  static char Buffer[512];

  for (;;) {
    mpu6050_read_data(&mpu6050_data, &qmc5883_data); // blocking ? TODO
    bmp_acquire_data(&bmp_pressure, &(sdata.bmp_temp), tp,
                     pp); // blocking ?                TODO

    sdata.alt = bmp280_get_altitude(bmp_pressure, p_ref, sdata.bmp_temp);
    // qmc5883_read_data(&qmc5883_data);
    sdata.heading = qmc5883_get_heading(&qmc5883_data, 108.8 / 1000.0);
    mpu_temp = mpu6050_calc_temp(mpu6050_data.temp);
    sdata.accel.accel_x = mpu6050_calc_accel(mpu6050_data.accel_x, ACCEL_FS_2G);
    sdata.accel.accel_y = mpu6050_calc_accel(mpu6050_data.accel_y, ACCEL_FS_2G);
    sdata.accel.accel_z = mpu6050_calc_accel(mpu6050_data.accel_z, ACCEL_FS_2G);
    sdata.gyro.gyro_x = mpu6050_calc_accel(mpu6050_data.gyro_x, FS_SEL_250);
    sdata.gyro.gyro_y = mpu6050_calc_accel(mpu6050_data.gyro_y, FS_SEL_250);
    sdata.gyro.gyro_z = mpu6050_calc_accel(mpu6050_data.gyro_z, FS_SEL_250);

    // // snprintf(SensorLog, sizeof(SensorLog), "%lu %ld %ld %ld %ld %ld %ld %d
    // %d %lu %d %d %d %d\r\n", timestamp, ftoi(accel.accel_x, ACC_DP),
    // ftoi(accel.accel_y, ACC_DP), ftoi(accel.accel_z, ACC_DP),
    // ftoi(gyro.gyro_x, GYR_DP), ftoi(gyro.gyro_y, GYR_DP), ftoi(gyro.gyro_z,
    // GYR_DP), (int16_t)mpu_temp, (int16_t)bmp_temp, (uint32_t)(alt*100.0),
    // qmc5883_data.MagX, qmc5883_data.MagY, qmc5883_data.MagZ,
    // (int16_t)heading); SensorLog[0] = sizeof(sdata); SensorLog[1] =
    // DATA_SENSORS; memcpy(SensorLog+2, &sdata, sizeof(sdata)); queue_status =
    // xQueueSend(xLogQueue, &SensorLog, 0);

    // SensorLog[0] = 12;
    // SensorLog[1] = DATA_DEBUG;
    // memcpy(SensorLog+2, &qmc5883_data, sizeof(qmc5883_data));
    // queue_status = xQueueSend(xLogQueue, &SensorLog, 0);

    // if (queue_status != pdPASS) {
    //     // handle queue full
    // }
    // vTaskGetRunTimeStats(Buffer);
    // SensorLog[0] = 250;
    // SensorLog[1] = DATA_DEBUG;
    // memcpy(SensorLog+2, Buffer, 250);
    // queue_status = xQueueSend(xLogQueue, &SensorLog, 0);
    // memcpy(SensorLog+2, Buffer+250, 250);
    // queue_status = xQueueSend(xLogQueue, &SensorLog, 0);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}