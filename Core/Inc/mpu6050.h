#pragma once

#include "stm32f4xx.h"

#define MPU6050_DEVICE_RESET_BIT 0x80
#define MPU6050_SLEEP_BIT 0x40
#define MPU6050_CYCLE_BIT 0x20
#define MPU6050_TEMP_DIS_BIT 0x08
#define MPU6050_DEVICE_RESET_BIT 0x80
#define MPU6050_STBY_XA_BIT 0x20
#define MPU6050_STBY_YA_BIT 0x10
#define MPU6050_STBY_ZA_BIT 0x08
#define MPU6050_STBY_XG_BIT 0x04
#define MPU6050_STBY_YG_BIT 0x02
#define MPU6050_STBY_ZG_BIT 0x01


typedef enum {
    CLKSEL_INT = 0,
    CLKSEL_PLLX,
    CLKSEL_PLLY,
    CLKSEL_PLLZ,
    CLKSEL_EXT32K,
    CLKSEL_EXT19M,
    CLKSEL_RESET = 7
} MPU6050_CLKSEL_OPTION;

typedef enum {
    ACCEL_FS_2G,
    ACCEL_FS_4G,
    ACCEL_FS_8G,
    ACCEL_FS_16G
} ACCEL_FS_OPTION;

typedef enum {
    FS_SEL_250,
    FS_SEL_500,
    FS_SEL_1000,
    FS_SEL_2000
} FS_SEL_OPTION;

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
} mpu6050_out;

typedef struct {
    float accel_x, accel_y, accel_z
} accel_3d;

typedef struct {
    float gyro_x, gyro_y, gyro_z
} gyro_3d;

/* Read MPU6050 register in blocking mode */
HAL_StatusTypeDef mpu6050_read_reg(uint8_t reg, uint8_t* value);

/* Write MPU6050 register */
HAL_StatusTypeDef mpu6050_write_reg(uint8_t reg, uint8_t data);

/* Check MPU6050 status*/
HAL_StatusTypeDef mpu6050_heartbeat();

HAL_StatusTypeDef mpu6050_set_power_options(uint8_t opt0, uint8_t opt1);

HAL_StatusTypeDef mpu6050_read_reg_burst(uint8_t reg, uint16_t data_size, uint8_t* value);

HAL_StatusTypeDef mpu6050_read_data(mpu6050_out* val);

float mpu6050_calc_temp(int16_t raw_temp);

float mpu6050_calc_gyro(int16_t raw_gyro, uint16_t scale);

float mpu6050_calc_accel(int16_t raw_accel, uint16_t scale);