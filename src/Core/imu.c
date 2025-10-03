#include "imu.h"
#include <math.h>

void imu_quat_get(const accel3d_t *acc, const gyro3d_t *gyro,
                  const mag3d_t *mag) {
  float yaw_raw, pitch_raw, roll_raw;
  yaw_raw = atan2f(acc->accel_x, acc->accel_z);
  pitch_raw = atan2f(acc->accel_y, acc->accel_z);
}