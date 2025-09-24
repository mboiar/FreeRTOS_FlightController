#include "mpu6050.h"
#include "i2c.h"


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

#define TIMEOUT 100


static I2C_HandleTypeDef* hi2c = &hi2c1;

HAL_StatusTypeDef mpu6050_read_reg(uint8_t reg, uint8_t* value) {
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef mpu6050_write_reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef mpu6050_heartbeat() {
    return HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR << 1, 3, TIMEOUT);
}

HAL_StatusTypeDef mpu6050_read_reg_burst(uint8_t reg, uint16_t data_size, uint8_t* value) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, value, data_size, TIMEOUT);
}

HAL_StatusTypeDef mpu6050_set_power_options(uint8_t opt0, uint8_t opt1) {
    mpu6050_write_reg(MPU6050_PWR_MGMT_1, opt0);
    return mpu6050_write_reg(MPU6050_PWR_MGMT_2, opt1);
}

HAL_StatusTypeDef mpu6050_read_data(mpu6050_out* val, qmc5883_out* mag_val) {
    HAL_StatusTypeDef status;
    if (mag_val != NULL) { // magnetometer data requested
        uint8_t rx_data[20] = {0};
        status = mpu6050_read_reg_burst(MPU6050_ACCEL_XOUT_H, 20, rx_data);
        val->accel_x = (rx_data[0]<<8) | rx_data[1];
        val->accel_y = (rx_data[2]<<8) | rx_data[3];
        val->accel_z = (rx_data[4]<<8) | rx_data[5];
        val->gyro_x = (rx_data[6]<<8) | rx_data[7];
        val->gyro_y = (rx_data[8]<<8) | rx_data[9];
        val->gyro_z = (rx_data[10]<<8) | rx_data[11];
        val->temp = (rx_data[12]<<8) | rx_data[13];
        mag_val->MagX = (rx_data[15]<<8) | rx_data[14];
        mag_val->MagY = (rx_data[17]<<8) | rx_data[16];
        mag_val->MagZ = (rx_data[19]<<8) | rx_data[18];
    } else {
        uint8_t rx_data[14] = {0};
        status = mpu6050_read_reg_burst(MPU6050_ACCEL_XOUT_H, 14, rx_data);
        val->accel_x = (rx_data[0]<<8) | rx_data[1];
        val->accel_y = (rx_data[2]<<8) | rx_data[3];
        val->accel_z = (rx_data[4]<<8) | rx_data[5];
        val->gyro_x = (rx_data[6]<<8) | rx_data[7];
        val->gyro_y = (rx_data[8]<<8) | rx_data[9];
        val->gyro_z = (rx_data[10]<<8) | rx_data[11];
        val->temp = (rx_data[12]<<8) | rx_data[13];
    }
    return status;
}

float mpu6050_calc_temp(int16_t raw_temp) {
    return (float)raw_temp/340.0 + 36.53;
}

float mpu6050_calc_accel(int16_t raw_accel, uint16_t scale) {
    return (float)raw_accel / (float)( 0x01 << (14 - scale) );
}

float mpu6050_calc_gyro(int16_t raw_gyro, uint16_t scale) {
    float fscale = 1;
    switch (scale) {
    case FS_SEL_250:
        fscale = 131.0;
        break;
    case FS_SEL_500:
        fscale = 65.5;
        break;
    case FS_SEL_1000:
        fscale = 32.8;
        break;
    case FS_SEL_2000:
        fscale = 16.4;
        break;
    default:
        break;
    }
    return (float)raw_gyro / fscale;
}

HAL_StatusTypeDef mpu6050_set_config(uint8_t cfg0, uint8_t cfg1) {
    HAL_StatusTypeDef status = mpu6050_write_reg(MPU6050_INT_PIN_CFG, cfg0);
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
    status = mpu6050_write_reg(MPU6050_I2C_SLV0_CTRL_REG, MPU6050_I2C_SLV0_EN | 6);
    return status;
}
