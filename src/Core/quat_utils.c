#include "quat_utils.h"

// Rotate vector by a quaternion.
inline void quat_rotate_vec(float dst[3], const float quat[4],
                            const float vec[3]) {
  float t1, t2, t3, v1, v2, v3;
  t1 = (quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] -
        quat[3] * quat[3]) *
       vec[0];
  t2 = 2 * (quat[1] * quat[2] - quat[0] * quat[3]) * vec[1];
  t3 = 2 * (quat[1] * quat[3] + quat[0] * quat[2]) * vec[2];
  v1 = t1 + t2 + t3;

  t1 = 2 * (quat[1] * quat[2] + quat[0] * quat[3]) * vec[0];
  t2 = (quat[0] * quat[0] - quat[1] * quat[1] + quat[2] * quat[2] -
        quat[3] * quat[3]) *
       vec[1];
  t3 = 2 * (quat[2] * quat[3] - quat[0] * quat[1]) * vec[2];
  v2 = t1 + t2 + t3;

  t1 = 2 * (quat[1] * quat[3] - quat[0] * quat[2]) * vec[0];
  t2 = 2 * (quat[2] * quat[3] + quat[0] * quat[1]) * vec[1];
  t3 = (quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] +
        quat[3] * quat[3]) *
       vec[2];
  v3 = t1 + t2 + t3;
  dst[0] = v1;
  dst[1] = v2;
  dst[2] = v3;
}

float clamp(float val, float min, float max) {
  if (val > max)
    return max;
  if (val < min)
    return min;
  return val;
}
