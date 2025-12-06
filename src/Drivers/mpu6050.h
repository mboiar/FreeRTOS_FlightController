#pragma once

#include "qmc5883.h"
#include "stm32f4xx.h"

#define MPU6050_DEVICE_RESET_BIT 0x01 << 7
#define MPU6050_SLEEP_BIT 0x01 << 6
#define MPU6050_CYCLE_BIT 0x01 << 5
#define MPU6050_TEMP_DIS_BIT 0x01 << 3
#define MPU6050_STBY_XA_BIT 0x01 << 5
#define MPU6050_STBY_YA_BIT 0x01 << 4
#define MPU6050_STBY_ZA_BIT 0x01 << 3
#define MPU6050_STBY_XG_BIT 0x01 << 2
#define MPU6050_STBY_YG_BIT 0x01 << 1
#define MPU6050_STBY_ZG_BIT 0x01

#define MPU6050_INT_LEVEL 0x01 << 7
#define MPU6050_INT_OPEN 0x01 << 6
#define MPU6050_I2C_BYPASS_EN 0x01 << 1
#define MPU6050_DATA_RDY_EN 0x01

#define MPU6050_FIFO_EN 0x01 << 6
#define MPU6050_I2C_MST_EN 0x01 << 5

#define MPU6050_WAIT_FOR_ES 0x01 << 6 //  sync with ext data

#define SMPRT_DIV 7U // 1 kHz Gyro+Accel sample rate

typedef enum {
  CLKSEL_INT = 0,
  CLKSEL_PLLX,
  CLKSEL_PLLY,
  CLKSEL_PLLZ,
  CLKSEL_EXT32K,
  CLKSEL_EXT19M,
  CLKSEL_RESET = 7
} MPU6050_CLKSEL_OPTION;

typedef enum { AFS_2G, AFS_4G, AFS_8G, AFS_16G } ACCEL_FS_OPTION;

typedef enum { FS_SEL_250, FS_SEL_500, FS_SEL_1000, FS_SEL_2000 } FS_SEL_OPTION;

typedef enum {
  LP_WAKE_1p25HZ = 0x00,
  LP_WAKE_5HZ = 0x40,
  LP_WAKE_20HZ = 0x80,
  LP_WAKE_40HZ = 0xC0
} MPU6050_LP_WAKE_CTRL_OPTION;

typedef struct {
  int16_t accel_x, accel_y, accel_z;
  int16_t gyro_x, gyro_y, gyro_z;
  int16_t temp;
} mpu6050_raw_t;

typedef struct {
  float accel_x, accel_y, accel_z;
} accel3d_t;

typedef struct {
  float gyro_x, gyro_y, gyro_z;
} gyro3d_t;

/* Read MPU6050 register in blocking mode */
HAL_StatusTypeDef mpu6050_read_reg(uint8_t reg, uint8_t *value);

HAL_StatusTypeDef mpu6050_set_gyro_accel_config(uint8_t fs_sel,
                                                uint8_t afs_sel);

/* Write MPU6050 register */
HAL_StatusTypeDef mpu6050_write_reg(uint8_t reg, uint8_t data);

/* Check MPU6050 status*/
HAL_StatusTypeDef mpu6050_heartbeat();

HAL_StatusTypeDef mpu6050_set_power_options(uint8_t opt0, uint8_t opt1);

HAL_StatusTypeDef mpu6050_read_reg_burst(uint8_t reg, uint16_t data_size,
                                         uint8_t *value);

HAL_StatusTypeDef mpu6050_read_data(accel3d_t *acc, gyro3d_t *gyro, float *temp,
                                    const gyro3d_t *offG, const accel3d_t *offA,
                                    float scaleA[3]);

float mpu6050_calc_temp(int16_t raw_temp);

float mpu6050_calc_gyro(int16_t raw_gyro, uint16_t scale);

float mpu6050_calc_accel(int16_t raw_accel, uint16_t scale);

HAL_StatusTypeDef mpu6050_set_config(uint8_t cfg0, uint8_t cfg1, uint8_t cfg2,
                                     uint8_t cfg3);

HAL_StatusTypeDef mpu6050_set_master_ctrl(uint8_t ctrl);

HAL_StatusTypeDef mpu6050_i2c_master_status(uint8_t *status);

HAL_StatusTypeDef mpu6050_user_ctrl(uint8_t ctrl);

HAL_StatusTypeDef mpu6050_slv0_init();

void IMU_RxCpltCallback();