#include "stm32f4xx.h"
#include "stdbool.h"

/* BMP280 Power mode */
typedef enum {
    BMP_SLEEP = 0,
    BMP_FORCED = 1,
    BMP_NORMAL = 3
} BMP_MODE;

/* BMP280 Status: measuring/imupdate/error */
typedef struct {
    bool BMP_MEASURING;  // =1 during conversion
    bool BMP_IMUPDATE;   // =1 when copying NVM data
    bool ERROR;
} BMP_STATUS;

/* Config params of BMP280 sensor */
typedef struct {
  uint8_t filter_coef;       // IIR filter coefficient
  uint8_t standby_time;      // Standby time
  bool spi3w_en;             // Enable 3-wire SPI
} BMP_CONFIG_PARAMS;

/* Data acquisition params of BMP280 sensor */
typedef struct {
  BMP_MODE mode;
  uint8_t temp_oversampling; // Temperature oversampling coefficient
  uint8_t pressure_oversampling; // Pressure oversampling coefficient
} BMP_CTRL_MEAS_PARAMS;

/* Temperature calibration params for BMP280 sensor */
typedef struct {
    unsigned short dig_T1;
    short dig_T2;
    short dig_T3;
} BMP_CAL_T_PARAMS;

/* Pressure calibration params for BMP280 sensor */
typedef struct {
    unsigned short dig_P1;
    short dig_P2;
    short dig_P3;
    short dig_P4;
    short dig_P5;
    short dig_P6;
    short dig_P7;
    short dig_P8;
    short dig_P9;
} BMP_CAL_P_PARAMS;

/* Check BMP280 status*/
HAL_StatusTypeDef bmp_heartbeat();

/* Initialize BMP280 sensor */
HAL_StatusTypeDef bmp_init(BMP_CAL_T_PARAMS* tp, BMP_CAL_P_PARAMS* pp, const BMP_CTRL_MEAS_PARAMS ctrl_p, const BMP_CONFIG_PARAMS conf_p);

/* Get BMP280 status */
BMP_STATUS bmp_get_status();

/* Reset BMP280 state */
HAL_StatusTypeDef bmp_reset();

/* Set data acquisition parameters */
HAL_StatusTypeDef bmp_ctrl_meas(const BMP_CTRL_MEAS_PARAMS p);

/* Set configuration parameters */
HAL_StatusTypeDef bmp_set_config(const BMP_CONFIG_PARAMS p);

/* Read calibration parameters from NVM */
HAL_StatusTypeDef bmp_read_calib_reg(BMP_CAL_T_PARAMS* tp, BMP_CAL_P_PARAMS* pp);

/* Read temperature (*C) and pressure (Pa) from BMP280 sensor */
HAL_StatusTypeDef bmp_acquire_data(float* press, float* temp, const BMP_CAL_T_PARAMS tp, const BMP_CAL_P_PARAMS pp);
