#pragma once

#include "i2c.h"
#include "math.h"

#define QMC5883_ID_REG 0x0D
#define QMC_5883_DATAX_LSB_REG 0x00
#define QMC_5883_DATAX_MSB_REG 0x01
#define QMC_5883_DATAY_LSB_REG 0x02
#define QMC_5883_DATAY_MSB_REG 0x03
#define QMC_5883_DATAZ_LSB_REG 0x04
#define QMC_5883_DATAZ_MSB_REG 0x05
#define QMC5883_STATUS_REG 0x06
#define QMC5883_DATAT_LSB_REG 0x07
#define QMC5883_DATAT_MSB_REG 0x08
#define QMC5883_MODE_REG 0x09
#define QMC5883_CTRL_REG 0x0A
#define QMC5883_RESET 0x80
#define QMC_5883_SR_REG 0x0B
#define QMC_5883_SR 0x01

#define QMC_5883_DRDY_BIT 0x01     // data ready
#define QMC_5883_OVL_BIT 0x01 << 1 // overflow flag
#define QMC_5883_DOR_BIT 0x01 << 2 // data skip

typedef enum { QMC5883_STANDBY, QMC5883_CONTINUOUS } QMC5883_MODE_OPTION;

typedef enum {
  ODR_10HZ,
  ODR_50HZ = 0x01 << 2,
  ODR_100HZ,
  ODR_200HZ
} QMC5883_ODR_OPTION;

typedef enum { RNG_2G, RNG_8G = 0x01 << 4 } QMC5883_RNG_OPTION;

typedef enum { INT_ENABLE, INT_DISABLE } QMC5883_INT_OPTION;

typedef enum {
  ROL_PNT_NORMAL,
  ROL_PNT_ENABLE = 0x01 << 6
} QMC5883_ROL_PNT_OPTION;

typedef enum {
  SOFT_RST_NORMAL,
  SOFT_RST_ENABLE = 0x01 << 7
} QMC5883_SOFT_RST_OPTION;

typedef enum {
  OSR_512,
  OSR_256 = 0x01 << 6,
  OSR_128,
  OSR_64,
} QMC5883_OSR_OPTION;

/* Config params of QMC5883 sensor */
typedef struct {
  QMC5883_MODE_OPTION MODE; // operational mode
  QMC5883_ODR_OPTION ODR;   // output data update rate
  QMC5883_RNG_OPTION RNG;   // range/sensitivity
  QMC5883_OSR_OPTION OSR;   // oversampling rate
} QMC5883_CONFIG_PARAMS;

/* Control params of QMC5883 sensor */
typedef struct {
  QMC5883_INT_OPTION INT_ENB;       // Interrupt Pin enable
  QMC5883_ROL_PNT_OPTION ROL_PNT;   // Point roll over function enable
  QMC5883_SOFT_RST_OPTION SOFT_RST; // Soft reset
} QMC5883_CTRL_PARAMS;

typedef struct {
  int16_t MagX, MagY, MagZ;
} qmc5883_out;

/* Check MPU6050 status*/
HAL_StatusTypeDef qmc5883_heartbeat();

HAL_StatusTypeDef qmc5883_set_ctrl(uint8_t ctrl);

HAL_StatusTypeDef qmc5883_read_data(qmc5883_out *val);

HAL_StatusTypeDef qmc5883_set_config(uint8_t cfg);

HAL_StatusTypeDef qmc5883_standby();

HAL_StatusTypeDef qmc5883_reset();

HAL_StatusTypeDef qmc5883_status();

float qmc5883_data_convert(int16_t val);

float qmc5883_get_heading(const qmc5883_out *data, float decl);