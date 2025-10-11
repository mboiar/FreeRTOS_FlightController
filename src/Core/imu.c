// Error-State Kalman Filter implementation optimized for ARM Cortex-M.
// Based on Sola et al. arXiv:1711.02508
//
// Author: Maks Boiar

#include "imu.h"
#include <math.h>
#include <memory.h>

// float acc_noise = 1e-3f;  // m/s^2/sqrt(s)
// float gyro_noise = 1e-4f; // rad/s/sqrt(s)
// float acc_psd = 400;      // ug/sqrt(Hz)
// float gyro_rnsd = 0.005;  // deg/s/sqrt(Hz)

// Rotate vector by a quaternion.
static inline void quat_rotate_vec(float dst[3], const float quat[4],
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

// multiply two quaternions
static inline void quat_mul(float dst[4], const float quat1[4],
                            const float quat2[4]) {
  float t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16;
  t1 = quat1[0] * quat2[0];
  t2 = quat1[1] * quat2[1];
  t3 = quat1[2] * quat2[2];
  t4 = quat1[3] * quat2[3];
  t5 = quat1[1] * quat2[2];
  t6 = quat1[0] * quat2[3];
  t7 = quat1[1] * quat2[3];
  t8 = quat1[0] * quat2[2];
  t9 = quat1[2] * quat2[3];
  t10 = quat1[0] * quat2[1];
  t11 = quat1[2] * quat2[1];
  t12 = quat1[1] * quat2[0];
  t13 = quat1[3] * quat2[2];
  t14 = quat1[2] * quat2[0];
  t15 = quat1[3] * quat2[1];
  t16 = quat1[3] * quat2[0];
  dst[0] = t1 - t2 - t3 - t4;
  dst[1] = t10 + t12 + t9 - t13;
  dst[2] = t8 - t7 + t14 + t15;
  dst[3] = t6 + t5 - t11 + t16;
}

static inline void rot_to_quat(float dst[4], const float rot[3]) {
  float m = rot[0] * rot[0] + rot[1] * rot[1] + rot[2] * rot[2];
  float theta = sqrtf(m);

  // small-angle approximation -> justified? no
  dst[0] = cosf(theta / 2);
  dst[1] = sinf(theta / 2) * rot[0];
  dst[2] = sinf(theta / 2) * rot[1];
  dst[3] = sinf(theta / 2) * rot[2];
}

static inline void quat_reset(float quat[4]) {
  quat[0] = 1;
  quat[1] = 0;
  quat[2] = 0;
  quat[3] = 0;
}

// !!! assume unit quaternion
static inline void quat_inv(float dst[4], float src[4]) {
  dst[0] = src[0];
  dst[1] = -src[1];
  dst[2] = -src[2];
  dst[3] = -src[3];
}

// !!! assume unit quaternion
static inline void quat_to_vec(float dst[3], float src[4]) {

  // TODO: guard against small angle divergence
  float m = 2 * atan2f(1, src[0]);
  dst[0] = src[1] * m;
  dst[1] = src[2] * m;
  dst[2] = src[3] * m;
}

// Linear combination of 2 vectors.
static inline void linv3(float dst[3], const float v0[3], const float v1[3],
                         float a, float b) {
  dst[0] = a * v0[0] + b * v1[0];
  dst[1] = a * v0[1] + b * v1[1];
  dst[2] = a * v0[2] + b * v1[2];
}

// Normalize quaternion
static inline void quat_norm(float src[4]) {
  float m = sqrtf(src[0] * src[0] + src[1] * src[1] + src[2] * src[2] +
                  src[3] * src[3]);
  if (m > 0) {
    src[0] = src[0] / m;
    src[1] = src[1] / m;
    src[2] = src[2] / m;
    src[3] = src[3] / m;
  }
}

void eskf_predict(eskf_t *eskf, const accel3d_t *acc_m, const gyro3d_t *gyro_m,
                  const float dt) {
  // assume acceleration is in g
  // assume gyro is in dps

  // update nominal state
  float acc_glob[3];  // acceleration in global frame
  float acc_body[3];  // acceleration in body frame
  float gyro_body[3]; // angular velocity in body frame
  float gyro_glob[3]; // angular velocity in global frame
  float gyro_quat[4];

  acc_body[0] = acc_m->accel_x;
  acc_body[1] = acc_m->accel_y;
  acc_body[2] = acc_m->accel_z;
  gyro_body[0] = gyro_m->gyro_x;
  gyro_body[1] = gyro_m->gyro_y;
  gyro_body[2] = gyro_m->gyro_z;
  linv3(acc_body, acc_body, eskf->state.acc_b, 9.81, -1);
  linv3(gyro_body, gyro_body, eskf->state.gyro_b, dt, -dt);
  quat_rotate_vec(acc_glob, eskf->state.quat, acc_body);
  quat_rotate_vec(gyro_glob, eskf->state.quat, gyro_body);
  rot_to_quat(gyro_quat, gyro_glob);
  quat_mul(eskf->state.quat, eskf->state.quat, gyro_quat);
  quat_norm(eskf->state.quat);

  // integrate nominal kinematics
  for (int i = 0; i < 3; i++) {
    eskf->state.pos[i] += eskf->state.vel[i] * dt +
                          0.5 * dt * dt * (acc_glob[i] + eskf->state.g[i]);
    eskf->state.vel[i] += dt * (acc_glob[i] + eskf->state.g[i]);
  }

  // update covariance
  // eskf->P = F * eskf->P * F.T eskf->P;
  // eskf->P
  // TODO

  // update error state (mean is initialized to 0, so not necessary)
  // dx = Fx * dx
}

void eskf_init(eskf_t *eskf, float sigma_an, float sigma_wn, float sigma_aw,
               float sigma_ww) {
  eskf_state_t st = {.pos = {0, 0, 0},
                     .vel = {0, 0, 0},
                     .quat = {1, 0, 0, 0},
                     .acc_b = {0, 0, 0},
                     .gyro_b = {0, 0, 0},
                     .g = {0, 0, -9.81}};
  eskf->state = st;
  memset(eskf->P, 0, sizeof(eskf->P));
  eskf->sigma_an = sigma_an;
  eskf->sigma_aw = sigma_aw;
  eskf->sigma_wn = sigma_wn;
  eskf->sigma_ww = sigma_ww;
}

// Note: more general and efficient update for small angle error:
// `dx.quat = state.quat*-1 * yaw_quat`
void eskf_update_yaw(eskf_t *eskf, float yaw_m, float cov) {
  // assume yaw is given in body frame
  // rotate back to NED
  float yaw_v[3] = {0, 0, yaw_m};
  float quat_v[3];
  float dx_yaw[3] = {0, 0, 0};
  quat_rotate_vec(yaw_v, eskf->state.quat, yaw_v);
  // conjugate ??
  quat_to_vec(quat_v, eskf->state.quat);

  // compute kalman gain
  // TODO

  // compute error state mean change (only yaw affected)
  dx_yaw[3] = K * (yaw_v[3] - quat_v[3]);
  rot_to_quat(eskf->dx.quat, dx_yaw);

  // update covariance
  // TODO

  // inject error-state
  quat_mul(eskf->state.quat, eskf->state.quat, eskf->dx.quat);
  quat_norm(eskf->state.quat);

  // reset error-state mean
  quat_reset(eskf->dx.quat);
}

void eskf_update_alt(eskf_t *eskf, float alt, float cov) {

  // compute kalman gain
  // TODO

  // observation
  eskf->dx.pos[2] = K * (alt - eskf->state.pos[2]);

  // update covariance
  // TODO

  // inject error-state
  linv3(eskf->state.pos, eskf->state.pos, eskf->dx.pos, 1, 1);

  // reset error-state mean
  memset(&eskf->dx.pos, 0, sizeof(eskf->dx.pos));
}

void eskf_reset(eskf_t *eskf) { memset(&eskf->dx, 0, sizeof(eskf->dx)); }

void eskf_inject(eskf_t *eskf, const eskf_state_t *es) {
  linv3(eskf->state.pos, eskf->state.pos, es->pos, 1, 1);
  linv3(eskf->state.vel, eskf->state.vel, es->vel, 1, 1);
  quat_mul(eskf->state.quat, eskf->state.quat, es->quat);
  linv3(eskf->state.acc_b, eskf->state.acc_b, es->acc_b, 1, 1);
  linv3(eskf->state.gyro_b, eskf->state.gyro_b, es->gyro_b, 1, 1);
}