#include "mpu6050.h"
#include "FreeRTOS.h"
#include "i2c.h"
#include "task.h"

// Register map
#define MPU6050_ADDR 0x68
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
// #define MPU6050_ACCEL_FS
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38

#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_FS_SEL_OFFSET 3
#define MPU6050_AFS_SEL_OFFSET 3

#define MPU6050_MASTER_CTRL 0x24
#define MPU6050_EXT_SENS_DATA_00 0x49
#define MPU6050_I2C_MASTER_STATUS_REG 0x36
#define MPU6050_I2C_SLV0_ADDR_REG 0x25
#define MPU6050_I2C_SLV0_REG 0x26
#define MPU6050_I2C_SLV0_CTRL_REG 0x27

#define MPU6050_I2C_SLV0_READ 0x1 << 7
#define MPU6050_I2C_SLV0_WRITE 0x0
#define MPU6050_I2C_SLV0_EN 0x01 << 7
#define MPU6050_I2C_SLV0_DO 0x99
#define MPU6050_I2C_SLV0_BYTE_SW 0x01 << 6
#define MPU6050_I2C_SLV0_REG_DIS 0x01 << 5
#define MPU6050_I2C_SLV0_GRP 0x01 << 4

#define MPU6050_SMPRT_DIV_REG 0x19

#define TIMEOUT 100

static I2C_HandleTypeDef *hi2c = &hi2c1;

static TaskHandle_t xTaskToNotify = NULL;

HAL_StatusTypeDef mpu6050_read_reg(uint8_t reg, uint8_t *value) {
  return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                          value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef mpu6050_write_reg(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef mpu6050_heartbeat() {
  return HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR << 1, 3, TIMEOUT);
}

HAL_StatusTypeDef mpu6050_read_reg_burst(uint8_t reg, uint16_t data_size,
                                         uint8_t *value) {
  xTaskToNotify = xTaskGetCurrentTaskHandle();
  if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT,
                           value, data_size) != HAL_OK) {
    // TODO: handle error
  }
  return HAL_OK;
}

HAL_StatusTypeDef mpu6050_set_power_options(uint8_t opt0, uint8_t opt1) {
  mpu6050_write_reg(MPU6050_PWR_MGMT_1, opt0);
  return mpu6050_write_reg(MPU6050_PWR_MGMT_2, opt1);
}

HAL_StatusTypeDef mpu6050_set_gyro_accel_config(uint8_t fs_sel,
                                                uint8_t afs_sel) {
  uint8_t config = (fs_sel & 0x01) << MPU6050_FS_SEL_OFFSET;
  config |= (fs_sel & 0x02) << (MPU6050_FS_SEL_OFFSET + 1);
  if (mpu6050_write_reg(MPU6050_GYRO_CONFIG, config) != HAL_OK) {
    return HAL_ERROR;
  }
  config = (afs_sel & 0x01) << MPU6050_AFS_SEL_OFFSET;
  config |= (afs_sel & 0x02) << (MPU6050_AFS_SEL_OFFSET + 1);
  return mpu6050_write_reg(MPU6050_ACCEL_CONFIG, config);
}

HAL_StatusTypeDef mpu6050_read_data(accel3d_t *acc, gyro3d_t *gyro, float *temp,
                                    const gyro3d_t *offG, const accel3d_t *offA,
                                    float scaleA[3]) {
  HAL_StatusTypeDef status;
  uint32_t notif;
  mpu6050_raw_t val;
  uint8_t rx_data[14] = {0};
  status = mpu6050_read_reg_burst(MPU6050_ACCEL_XOUT_H, 14, rx_data);
  do {
    xTaskNotifyWait(pdFALSE, 0x8000, &notif, portMAX_DELAY);
  } while ((notif & 0x8000) == 0);

  notif &= ~0x8000;

  val.accel_x = (int16_t)(rx_data[0] << 8) | rx_data[1];
  val.accel_y = (int16_t)(rx_data[2] << 8) | rx_data[3];
  val.accel_z = (int16_t)(rx_data[4] << 8) | rx_data[5];
  val.temp = (int16_t)(rx_data[6] << 8) | rx_data[7];
  val.gyro_x = (int16_t)(rx_data[8] << 8) | rx_data[9];
  val.gyro_y = (int16_t)(rx_data[10] << 8) | rx_data[11];
  val.gyro_z = (int16_t)(rx_data[12] << 8) | rx_data[13];
  acc->accel_x =
      (mpu6050_calc_accel(val.accel_x, AFS_2G) - offA->accel_x) * scaleA[0];
  acc->accel_y =
      (mpu6050_calc_accel(val.accel_y, AFS_2G) - offA->accel_y) * scaleA[1];
  acc->accel_z =
      (mpu6050_calc_accel(val.accel_z, AFS_2G) - offA->accel_z) * scaleA[2];
  gyro->gyro_x =
      -1.0f * mpu6050_calc_gyro(val.gyro_x, FS_SEL_250); // - offG->gyro_x;
  gyro->gyro_y =
      -1.0f * mpu6050_calc_gyro(val.gyro_y, FS_SEL_250); // - offG->gyro_y;
  gyro->gyro_z =
      -1.0f * mpu6050_calc_gyro(val.gyro_z, FS_SEL_250); // - offG->gyro_z;
  *temp = mpu6050_calc_temp(val.temp);

  return HAL_OK;
}

float mpu6050_calc_temp(int16_t raw_temp) {
  return (float)raw_temp / 340.0f + 36.53f;
}

float mpu6050_calc_accel(int16_t raw_accel, uint16_t scale) {
  return (float)raw_accel / (float)(0x01 << (14 - scale));
}

float mpu6050_calc_gyro(int16_t raw_gyro, uint16_t scale) {
  float fscale = 1.0f;
  switch (scale) {
  case FS_SEL_250:
    fscale = 131.0f;
    break;
  case FS_SEL_500:
    fscale = 65.5f;
    break;
  case FS_SEL_1000:
    fscale = 32.8f;
    break;
  case FS_SEL_2000:
    fscale = 16.4f;
    break;
  default:
    break;
  }
  return (float)raw_gyro * M_PI / fscale / 180.0f;
}

HAL_StatusTypeDef mpu6050_set_config(uint8_t cfg0, uint8_t cfg1, uint8_t cfg2,
                                     uint8_t cfg_cfg) {
  mpu6050_write_reg(MPU6050_SMPRT_DIV_REG, cfg2);
  mpu6050_write_reg(MPU6050_INT_PIN_CFG, cfg0);
  mpu6050_write_reg(MPU6050_CONFIG, cfg_cfg);
  return mpu6050_write_reg(MPU6050_INT_ENABLE, cfg1);
}

HAL_StatusTypeDef mpu6050_user_ctrl(uint8_t ctrl) {
  return mpu6050_write_reg(MPU6050_USER_CTRL, ctrl);
}

HAL_StatusTypeDef mpu6050_set_master_ctrl(uint8_t ctrl) {
  return mpu6050_write_reg(MPU6050_MASTER_CTRL, ctrl);
}

HAL_StatusTypeDef mpu6050_i2c_master_status(uint8_t *status) {
  return mpu6050_read_reg(MPU6050_I2C_MASTER_STATUS_REG, status);
}

HAL_StatusTypeDef mpu6050_slv0_init() {
  uint8_t ctrl = QMC5883_ID_REG | MPU6050_I2C_SLV0_READ;
  HAL_StatusTypeDef status = mpu6050_write_reg(MPU6050_I2C_SLV0_ADDR_REG, ctrl);
  status = mpu6050_write_reg(MPU6050_I2C_SLV0_REG, QMC_5883_DATAX_LSB_REG);
  status =
      mpu6050_write_reg(MPU6050_I2C_SLV0_CTRL_REG, MPU6050_I2C_SLV0_EN | 6);
  return status;
}

void IMU_RxCpltCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  configASSERT(xTaskToNotify != NULL);
  xTaskNotifyFromISR(xTaskToNotify, 0x8000, eSetBits,
                     &xHigherPriorityTaskWoken);
  xTaskToNotify = NULL;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}