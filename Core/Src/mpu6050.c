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
#define MPU6050_ACCEL_FS 
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B


uint8_t mpu_read_reg(uint8_t reg) {
    uint8_t value = 0;
    HAL_Delay(30);
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, &value, sizeof(value), HAL_MAX_DELAY) != HAL_OK){
      Error_Handler();
    }
    HAL_Delay(30);
    return value;
}

HAL_StatusTypeDef mpu_heartbeat() {
    return mpu_read_reg(MPU6050_WHO_AM_I) == MPU6050_ADDR ? HAL_OK : HAL_ERROR;
}