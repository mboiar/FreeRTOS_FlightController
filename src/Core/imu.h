#pragma once

#include "bmp280.h"
#include "mpu6050.h"
#include "qmc5883.h"
#include "semphr.h"

typedef struct {
  accel_3d accel;
  gyro_3d gyro;
  float heading;
  float alt;
  // TickType_t timestamp;
  float bmp_temp;
} sensor_data_t;

extern sensor_data_t imu_data;
extern SemaphoreHandle_t imu_mutex;
