#include "qmc5883.h"

#define TIMEOUT 100

I2C_HandleTypeDef* hi2c = &hi2c1;

HAL_StatusTypeDef qmc5883_read_reg(uint8_t reg, uint8_t* value) {
    return HAL_I2C_Mem_Read(hi2c, QMC5883_ID_REG << 1, reg, I2C_MEMADD_SIZE_8BIT, value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef qmc5883_write_reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, QMC5883_ID_REG << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef qmc5883_heartbeat() {
    return HAL_I2C_IsDeviceReady(hi2c, QMC5883_ID_REG << 1, 3, TIMEOUT);
}

HAL_StatusTypeDef qmc5883_read_reg_burst(uint8_t reg, uint16_t data_size, uint8_t* value) {
    return HAL_I2C_Mem_Read(&hi2c1, QMC5883_ID_REG << 1, reg, I2C_MEMADD_SIZE_8BIT, value, data_size, TIMEOUT);
}

HAL_StatusTypeDef qmc5883_read_data(qmc5883_out* val) {
    int8_t rx_data[6] = {0};
    HAL_StatusTypeDef status = qmc5883_read_reg_burst(QMC_5883_DATAX_LSB_REG, 6, rx_data);
    if (status == HAL_OK) {
        val->MagX = (rx_data[1]<<8) | rx_data[0];
        val->MagY = (rx_data[3]<<8) | rx_data[2];
        val->MagZ = (rx_data[5]<<8) | rx_data[4];
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

HAL_StatusTypeDef qmc5883_status(uint8_t* status) {
    return qmc5883_read_reg(QMC5883_STATUS_REG, status);
}

float qmc5883_data_convert(int16_t val) {
    return (float) val / 32768.0 * 2;
}