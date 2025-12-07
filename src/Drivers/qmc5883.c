#include "qmc5883.h"

#define TIMEOUT 100

static I2C_HandleTypeDef *hi2c = &hi2c1;

HAL_StatusTypeDef qmc5883_read_reg(uint8_t reg, uint8_t *value) {
  return HAL_I2C_Mem_Read(hi2c, QMC5883_ID_REG << 1, reg, I2C_MEMADD_SIZE_8BIT,
                          value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef qmc5883_write_reg(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(hi2c, QMC5883_ID_REG << 1, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef qmc5883_heartbeat() {
  return HAL_I2C_IsDeviceReady(hi2c, QMC5883_ID_REG << 1, 3, TIMEOUT);
}

HAL_StatusTypeDef qmc5883_read_reg_burst(uint8_t reg, uint16_t data_size,
                                         uint8_t *value) {
  return HAL_I2C_Mem_Read(&hi2c1, QMC5883_ID_REG << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, value, data_size, TIMEOUT);
}

HAL_StatusTypeDef qmc5883_read_data(mag3d_t *mag, float offv[3],
                                    float offM[3][3]) {
  uint8_t rx_data[6] = {0};
  qmc5883_raw_t mag_val;
  HAL_StatusTypeDef status =
      qmc5883_read_reg_burst(QMC_5883_DATAX_LSB_REG, 6, rx_data); // TODO: IT
  if (status == HAL_OK) {
    // mag_val.MagZ = -(rx_data[1] << 8) | rx_data[0]; // local X data
    // mag_val.MagY = (rx_data[3] << 8) | rx_data[2];  // local Y data
    // mag_val.MagX = (rx_data[5] << 8) | rx_data[4];  // local Z data
    mag_val.MagX = (rx_data[1] << 8) | rx_data[0]; // local X data
    mag_val.MagY = (rx_data[3] << 8) | rx_data[2]; // local Y data
    mag_val.MagZ = (rx_data[5] << 8) | rx_data[4]; // local Z data

    mag->MagX = qmc5883_data_convert(mag_val.MagX) - offv[0];
    mag->MagY = qmc5883_data_convert(mag_val.MagY) - offv[1];
    mag->MagZ = qmc5883_data_convert(mag_val.MagZ) - offv[2];
    mag->MagX = offM[0][0] * mag->MagX + offM[0][1] * mag->MagY +
                offM[0][2] * mag->MagZ;
    mag->MagY = offM[1][0] * mag->MagX + offM[1][1] * mag->MagY +
                offM[1][2] * mag->MagZ;
    mag->MagZ = offM[2][0] * mag->MagX + offM[2][1] * mag->MagY +
                offM[2][2] * mag->MagZ;
  }

  return status;
}

HAL_StatusTypeDef qmc5883_set_config(uint8_t cfg) {
  qmc5883_write_reg(QMC_5883_SR_REG, QMC_5883_SR);
  return qmc5883_write_reg(QMC5883_MODE_REG, cfg);
}

HAL_StatusTypeDef qmc5883_set_ctrl(uint8_t ctrl) {
  return qmc5883_write_reg(QMC5883_CTRL_REG, ctrl);
}

HAL_StatusTypeDef qmc5883_standby() {
  return qmc5883_write_reg(QMC5883_MODE_REG, QMC5883_STANDBY);
}

HAL_StatusTypeDef qmc5883_reset() {
  return qmc5883_write_reg(QMC5883_CTRL_REG, QMC5883_RESET);
}

HAL_StatusTypeDef qmc5883_status(uint8_t *status) {
  return qmc5883_read_reg(QMC5883_STATUS_REG, status);
}

float qmc5883_data_convert(int16_t val) { return ((float)val) / 32768.0 * 2; }

float qmc5883_get_heading(const mag3d_t *data, float decl) {
  float heading = atan2f(data->MagY, data->MagX);
  heading += decl;
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  if (heading > 2 * M_PI) {
    heading -= 2 * M_PI;
  }
  return heading;
}