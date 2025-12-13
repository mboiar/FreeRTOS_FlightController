// Error-State Kalman Filter implementation optimized for ARM Cortex-M.
// Based on Sola et al. arXiv:1711.02508
//
// Author: Maks Boiar

#pragma once

#include "bmp280.h"
#include "gps.h"
#include "mpu6050.h"
#include "qmc5883.h"

#define MAG_DECL 0.109665f
#define MAG_INCL 1.162681f

typedef struct {
  accel3d_t accel;
  gyro3d_t gyro;
  float heading;
  float alt;
  // TickType_t timestamp;
  float bmp_temp;
} sensor_data_t;

typedef struct {
  float pos[3];    // position (NED)
  float vel[3];    // velocity (NED)
  float quat[4];   // orientation quaternion (body->NED)
  float acc_b[3];  // accelerometer bias
  float gyro_b[3]; // gyro bias
  float g[3];      // gravity (assume constant)
} eskf_state_t;

typedef struct {
  float pos[3];    // position (NED)
  float vel[3];    // velocity (NED)
  float theta[3];  // orientation (Euler angles)
  float acc_b[3];  // accelerometer bias
  float gyro_b[3]; // gyro bias
} eskf_err_state_t;

typedef struct {
  eskf_state_t state; // Nominal state
  float dx[15];       // Error state
  float sigma_an;
  float sigma_wn;
  float sigma_aw;
  float sigma_ww;
} eskf_t;

/**
 * @brief Initialize Error State Kalman Filter.
 * @param eskf `eskf_t` instance being initialized
 * @retval None
 */
void eskf_init(eskf_t *eskf, float sigma_an, float sigma_wn, float sigma_aw,
               float sigma_ww, gyro3d_t *gyro_bm, mag3d_t *mag_init,
               accel3d_t *accel_init);

/**
 * @brief ESKF prediction step.
 * @param acc_m acceleration measurement, normalized
 * @param gyro_m gyroscope measurement ?
 * @retval None
 */
void eskf_predict(eskf_t *eskf, const accel3d_t *acc_m, const gyro3d_t *gyro_m,
                  const float dt);

/**
 * @brief ESKF update using magnetometer measurement.
 * @retval None
 */
void eskf_update_yaw(eskf_t *eskf, mag3d_t *mag, float cov);

/**
 * @brief ESKF update using barometer measurement.
 * @param baro_m barometer measurement in [m]
 * @retval None
 */
void eskf_update_baro(eskf_t *eskf, float alt, float cov);

void eskf_update_dist_sensor(eskf_t *eskf, float alt, float cov);

void eskf_update_gps(eskf_t *eskf, const GPS_data *data, uint64_t home_lon,
                     uint64_t home_lat, float home_alt);

void eskf_get_cov_orientation(eskf_t *eskf, float dst[9]);

void eskf_get_cov_posvel(eskf_t *eskf, float dst[15]);

void eskf_get_cov_pos(eskf_t *eskf, float dst[9]);
