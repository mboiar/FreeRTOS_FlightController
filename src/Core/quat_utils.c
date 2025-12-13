#include "quat_utils.h"

// Rotate vector by a quaternion.
inline void quat_rotate_vec(float dst[3], const float quat[4],
                            const float vec[3]) {
  float t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
  t1 = quat[0] * quat[0];
  t2 = quat[1] * quat[1];
  t3 = quat[2] * quat[2];
  t4 = quat[3] * quat[3];
  t5 = quat[1] * quat[2];
  t6 = quat[0] * quat[3];
  t7 = quat[1] * quat[3];
  t8 = quat[0] * quat[2];
  t9 = quat[2] * quat[3];
  t10 = quat[0] * quat[1];
  dst[0] = (t1 + t2 - t3 - t4) * vec[0] + 2 * (t5 - t6) * vec[1] +
           2 * (t7 + t8) * vec[2];
  dst[1] = 2 * (t5 + t6) * vec[0] + (t1 - t2 + t3 - t4) * vec[1] +
           2 * (t9 - t10) * vec[2];
  dst[2] = 2 * (t7 - t8) * vec[0] + 2 * (t9 + t10) * vec[1] +
           (t1 - t2 - t3 + t4) * vec[2];
}
