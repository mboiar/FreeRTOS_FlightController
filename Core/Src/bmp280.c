#include "bmp280.h"
#include "i2c.h"
#include "stdbool.h"
#include "stdio.h"

// Register map
#define BMP280_ID 0xD0
#define BMP280_ID_VALUE 0x58
#define BMP280_ADDR 0x76

#define BMP280_TEMP_X_LSB 0xFC
#define BMP280_TEMP_LSB 0xFB
#define BMP280_TEMP_MSB 0xFA
#define BMP280_PRESS_X_LSB 0xF9
#define BMP280_PRESS_LSB 0xF8
#define BMP280_PRESS_MSB 0xF7
#define BMP280_CONFIG 0xF5
#define BMP280_CTRL_MEAS 0xF4
#define BMP280_STATUS 0xF3
#define BMP280_RESET 0xE0

// Calibration parameter registers (read-only)
#define BMP280_DIG_T1_LSB 0x88
#define BMP280_DIG_T1_MSB 0x89
#define BMP280_DIG_T2_LSB 0x8A
#define BMP280_DIG_T2_MSB 0x8B
#define BMP280_DIG_T3_LSB 0x8C
#define BMP280_DIG_T3_MSB 0x8D
#define BMP280_DIG_P1_LSB 0x8E
#define BMP280_DIG_P1_MSB 0x8F
#define BMP280_DIG_P2_LSB 0x90
#define BMP280_DIG_P2_MSB 0x91
#define BMP280_DIG_P3_LSB 0x92
#define BMP280_DIG_P3_MSB 0x93
#define BMP280_DIG_P4_LSB 0x94
#define BMP280_DIG_P4_MSB 0x95
#define BMP280_DIG_P5_LSB 0x96
#define BMP280_DIG_P5_MSB 0x97
#define BMP280_DIG_P6_LSB 0x98
#define BMP280_DIG_P6_MSB 0x99
#define BMP280_DIG_P7_LSB 0x9A
#define BMP280_DIG_P7_MSB 0x9B
#define BMP280_DIG_P8_LSB 0x9C
#define BMP280_DIG_P8_MSB 0x9D
#define BMP280_DIG_P9_LSB 0x9E
#define BMP280_DIG_P9_MSB 0x9F

#define BMP280_RESET_VAL 0xB6

#define TIMEOUT 100

typedef long signed int BMP280_S32_t;
typedef long unsigned int BMP280_U32_t;

HAL_StatusTypeDef bmp_read_reg_burst(uint8_t reg, uint16_t data_size, uint8_t* value) {
    return HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, value, data_size, TIMEOUT);
}

HAL_StatusTypeDef bmp_read_reg(uint8_t reg, uint8_t* value) {
  return bmp_read_reg_burst(reg, sizeof(value), value);
}

HAL_StatusTypeDef bmp_heartbeat() {
    return HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR << 1, 1, TIMEOUT);
}

HAL_StatusTypeDef bmp_write_reg(uint8_t reg, uint8_t value) {
    // HAL_Delay(100);
    return HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef bmp_init(BMP_CAL_T_PARAMS* tp, BMP_CAL_P_PARAMS* pp, const BMP_CTRL_MEAS_PARAMS ctrl_p, const BMP_CONFIG_PARAMS conf_p) {
  if (bmp_heartbeat() != HAL_OK) {
    printf("BMP280: No heartbeat");
    return HAL_ERROR;
  }
  if (bmp_read_calib_reg(tp, pp) != HAL_OK) {
    return HAL_ERROR;
  }

  bmp_reset();
  HAL_Delay(10);
  if (bmp_set_config(conf_p) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(10);
  if (bmp_ctrl_meas(ctrl_p) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T, BMP280_S32_t* t_fine, const BMP_CAL_T_PARAMS tp) {
  BMP280_S32_t var1, var2, T;
  var1 = ((((adc_T>>3) - ((BMP280_S32_t)tp.dig_T1<<1))) * ((BMP280_S32_t)tp.dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((BMP280_S32_t)tp.dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)tp.dig_T1))) >> 12) * ((BMP280_S32_t)tp.dig_T3)) >> 14;
  *t_fine = var1 + var2;
  T = (*t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “96386” represents 96386 Pa = 963.86 hPa
BMP280_U32_t bmp280_compensate_P_int32(BMP280_S32_t adc_P, BMP280_S32_t t_fine, const BMP_CAL_P_PARAMS pp) {
  BMP280_S32_t var1, var2;
  BMP280_U32_t p;
  var1 = (((BMP280_S32_t)t_fine)>>1) - (BMP280_S32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((BMP280_S32_t)pp.dig_P6);
  var2 = var2 + ((var1*((BMP280_S32_t)pp.dig_P5))<<1);
  var2 = (var2>>2)+(((BMP280_S32_t)pp.dig_P4)<<16);
  var1 = (((pp.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((BMP280_S32_t)pp.dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((BMP280_S32_t)pp.dig_P1))>>15);
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = (((BMP280_U32_t)(((BMP280_S32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((BMP280_U32_t)var1);
  } else {
    p = (p / (BMP280_U32_t)var1) * 2;
  }
  var1 = (((BMP280_S32_t)pp.dig_P9) * ((BMP280_S32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((BMP280_S32_t)(p>>2)) * ((BMP280_S32_t)pp.dig_P8))>>13;
  p = (BMP280_U32_t)((BMP280_S32_t)p + ((var1 + var2 + pp.dig_P7) >> 4));
  return p;
}

HAL_StatusTypeDef bmp_reset() {
  return bmp_write_reg(BMP280_RESET, BMP280_RESET_VAL);
}

BMP_STATUS bmp_get_status() {
  BMP_STATUS status;
  uint8_t val;
  if (bmp_read_reg(BMP280_STATUS, &val) != HAL_OK) {
    status.ERROR = 1;
    return status;
  }
  status.BMP_IMUPDATE = val & 0;
  status.BMP_MEASURING = val & 4;
  return status;
}

HAL_StatusTypeDef bmp_ctrl_meas(const BMP_CTRL_MEAS_PARAMS p) {
  uint8_t osrst = p.temp_oversampling, osrsp = p.pressure_oversampling, mode = p.mode;
  uint8_t val = mode | (osrsp << 2) | (osrst << 5);
  return bmp_write_reg(BMP280_CTRL_MEAS, val);
}

HAL_StatusTypeDef bmp_set_config(const BMP_CONFIG_PARAMS p) {
  uint8_t t_sb = p.standby_time, filter = p.filter_coef, spi3w_en = p.spi3w_en;
  uint8_t val = spi3w_en | (filter << 2) | (t_sb << 5);
  return bmp_write_reg(BMP280_CONFIG, val);
}

HAL_StatusTypeDef bmp_read_data_raw(BMP280_S32_t* press, BMP280_S32_t* temp) {
  uint8_t data[6];
  *press = 0;
  *temp = 0;
   if (bmp_read_reg_burst(BMP280_PRESS_MSB, 6, data) != HAL_OK) {
    return HAL_ERROR;
   }
   *press = (data[0] << 16) | (data[1] << 8) | data[2];
   *temp = (data[3] << 16) | (data[4] << 8) | data[5];
   return HAL_OK;
}

HAL_StatusTypeDef bmp_read_calib_reg(BMP_CAL_T_PARAMS* tp, BMP_CAL_P_PARAMS* pp) {
  uint8_t data[24];
  if (bmp_read_reg_burst(BMP280_DIG_T1_LSB, 24, data) != HAL_OK) {
    return HAL_ERROR;
  }
  BMP_CAL_T_PARAMS tp_val = {
    .dig_T1 = data[0] | data[1] << 8,
    .dig_T2 = data[2] | data[3] << 8,
    .dig_T3 = data[4] | data[5] << 8,

  };
  *tp = tp_val;
  BMP_CAL_P_PARAMS pp_val = {
    .dig_P1 = data[6] | data[7] << 8,
    .dig_P2 = data[8] | data[9] << 8,
    .dig_P3 = data[10] | data[11] << 8,
    .dig_P4 = data[12] | data[13] << 8,
    .dig_P5 = data[14] | data[15] << 8,
    .dig_P6 = data[16] | data[17] << 8,
    .dig_P7 = data[18] | data[19] << 8,
    .dig_P8 = data[20] | data[21] << 8,
    .dig_P9 = data[22] | data[23] << 8,
  };
  *pp = pp_val;

  return HAL_OK;
}

HAL_StatusTypeDef bmp_acquire_data(float* press, float* temp, const BMP_CAL_T_PARAMS tp, const BMP_CAL_P_PARAMS pp) {
  BMP280_S32_t temp_fixed, press_fixed;
  if (bmp_read_data_raw(&press_fixed, &temp_fixed) != HAL_OK) {
    return HAL_ERROR;
  }
  BMP280_S32_t t_fine = 0;
  bmp280_compensate_T_int32(temp_fixed, &t_fine, tp);
  bmp280_compensate_P_int32(press_fixed, t_fine, pp);
  *press = (float)press_fixed;
  *temp = (float)temp_fixed / 100.0;

  return HAL_OK;
}