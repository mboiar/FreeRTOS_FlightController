// Error-State Kalman Filter implementation optimized for ARM Cortex-M.
// Based on Sola et al. arXiv:1711.02508
//
// Author: Maks Boiar

#include "imu.h"
#include <math.h>
#include <memory.h>
#include <stdint.h>

#include "arm_math.h"
#include "dsp/fast_math_functions.h"
#include "dsp/matrix_functions.h"

// float acc_noise = 1e-3f;  // m/s^2/sqrt(s)
// float gyro_noise = 1e-4f; // rad/s/sqrt(s)
// float acc_psd = 400;      // ug/sqrt(Hz)
// float gyro_rnsd = 0.005;  // deg/s/sqrt(Hz)

static inline void euler_to_quat(float q[4], float roll, float pitch,
                                 float yaw);

static arm_matrix_instance_f32 F, P, Ft, Q, Ra, R, W, HPH, K, H, Ht, Pv;
static float Ft_data[15 * 15], F_data[15 * 15], Q_data[15 * 15], K_data[4 * 4],
    HPH_data[3 * 3], H_data[3 * 3], Ht_data[3 * 3], P_data[3 * 3];
static float acc_glob[3];                    // acceleration in global frame
static float acc_body[3];                    // acceleration in body frame
static float gyro_body[3], gyro_body_int[3]; // angular velocity in body frame
static float gyro_quat[4];
static float a_x[9], Rq[9], wrot[9];

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

static inline void mat_set3x3(arm_matrix_instance_f32 *M, uint16_t r0,
                              uint16_t c0, const float m[9]) {
  float *p = M->pData + r0 * M->numCols + c0;
  const uint16_t N = M->numCols;

  p[0] = m[0];
  p[1] = m[1];
  p[2] = m[2];
  p[N + 0] = m[3];
  p[N + 1] = m[4];
  p[N + 2] = m[5];
  p[2 * N + 0] = m[6];
  p[2 * N + 1] = m[7];
  p[2 * N + 2] = m[8];
}

static inline void set3x3(float *dst, uint16_t r0, uint16_t c0,
                          const float m[9], uint16_t ncols) {
  float *p = dst + r0 * ncols + c0;

  p[0] = m[0];
  p[1] = m[1];
  p[2] = m[2];
  p[ncols + 0] = m[3];
  p[ncols + 1] = m[4];
  p[ncols + 2] = m[5];
  p[2 * ncols + 0] = m[6];
  p[2 * ncols + 1] = m[7];
  p[2 * ncols + 2] = m[8];
}

static inline void mat_set_diag(arm_matrix_instance_f32 *M, uint16_t r0,
                                uint16_t c0, float val) {
  float *p = M->pData + r0 * M->numCols + c0;
  const uint16_t N = M->numCols;

  p[0] = val;
  p[N + 1] = val;
  p[2 * N + 2] = val;
}

static inline void mat_add_diag(arm_matrix_instance_f32 *M, uint16_t r0,
                                uint16_t c0, float val) {
  float *p = M->pData + r0 * M->numCols + c0;
  const uint16_t N = M->numCols;

  p[0] += val;
  p[N + 1] += val;
  p[2 * N + 2] += val;
}
static inline void mat_scale3x3(arm_matrix_instance_f32 *M, uint16_t r0,
                                uint16_t c0, float val) {
  float *p = M->pData + r0 * M->numCols + c0;
  const uint16_t N = M->numCols;

  p[0] *= val;
  p[1] *= val;
  p[2] *= val;
  p[N + 0] *= val;
  p[N + 1] *= val;
  p[N + 2] *= val;
  p[2 * N + 0] *= val;
  p[2 * N + 1] *= val;
  p[2 * N + 2] *= val;
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
  float sinth = arm_sin_f32(theta / 2);

  // small-angle approximation -> justified? no
  dst[0] = arm_cos_f32(theta / 2);
  dst[1] = sinth * rot[0] / theta;
  dst[2] = sinth * rot[1] / theta;
  dst[3] = sinth * rot[2] / theta;
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
  float m;
  arm_atan2_f32(1, src[0], &m);
  dst[0] = src[1] * 2 * m;
  dst[1] = src[2] * 2 * m;
  dst[2] = src[3] * 2 * m;
}

static inline void quat_to_rot_mat(float dst[9], float quat[4]) {
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
  dst[0] = t1 + t2 - t3 - t4;
  dst[1] = 2 * (t5 - t6);
  dst[2] = 2 * (t7 + t8);
  dst[3] = 2 * (t5 + t6);
  dst[4] = t1 - t2 + t3 - t4;
  dst[5] = 2 * (t9 - t10);
  dst[6] = 2 * (t7 - t8);
  dst[7] = 2 * (t9 + t10);
  dst[8] = t1 - t2 - t3 + t4;
}

// Linear combination of 2 vectors.
static inline void linv3(float *dst, const float *v0, const float *v1, float a,
                         float b) {
  dst[0] = a * v0[0] + b * v1[0];
  dst[1] = a * v0[1] + b * v1[1];
  dst[2] = a * v0[2] + b * v1[2];
}

// Linear combination of 4 vectors.
static inline void linv34(float dst[3], const float v0[3], const float v1[3],
                          const float v2[3], const float v3[3], float a,
                          float b, float c, float d) {
  dst[0] = a * v0[0] + b * v1[0] + c * v2[0] + d * v3[0];
  dst[1] = a * v0[1] + b * v1[1] + c * v2[1] + d * v3[1];
  dst[2] = a * v0[2] + b * v1[2] + c * v2[2] + d * v3[2];
}

// Normalize quaternion
static inline void quat_norm(float src[4]) {
  float m;
  arm_sqrt_f32(src[0] * src[0] + src[1] * src[1] + src[2] * src[2] +
                   src[3] * src[3],
               &m);
  if (m > 0) {
    src[0] = src[0] / m;
    src[1] = src[1] / m;
    src[2] = src[2] / m;
    src[3] = src[3] / m;
  }
}

static inline void vec_skew(float dst[9], float src[3]) {
  dst[0] = 0;
  dst[1] = -src[2];
  dst[2] = src[1];
  dst[8] = 0;
  dst[3] = src[2];
  dst[6] = -src[1];
  dst[4] = 0;
  dst[5] = -src[0];
  dst[7] = src[0];
}

// Unroll Fx*P*Fx.T+Fi*Qi*Fi.T
static inline void update_P(float Pnew) {}

void eskf_predict(eskf_t *eskf, const accel3d_t *acc_m, const gyro3d_t *gyro_m,
                  const float dt) {
  // assume acceleration is in g
  // assume gyro is in dps

  // update nominal state

  acc_body[0] = acc_m->accel_x;
  acc_body[1] = acc_m->accel_y;
  acc_body[2] = acc_m->accel_z;
  gyro_body[0] = gyro_m->gyro_x;
  gyro_body[1] = gyro_m->gyro_y;
  gyro_body[2] = gyro_m->gyro_z;

  linv3(acc_body, acc_body, eskf->state.acc_b, 9.81f, -1);
  linv3(gyro_body_int, gyro_body, eskf->state.gyro_b, dt,
        -dt); // substract bias
  quat_rotate_vec(acc_glob, eskf->state.quat, acc_body);
  rot_to_quat(gyro_quat, gyro_body_int);
  quat_mul(eskf->state.quat, eskf->state.quat, gyro_quat);
  quat_norm(eskf->state.quat);

  // integrate nominal kinematics
  for (int i = 0; i < 3; i++) {
    eskf->state.pos[i] += eskf->state.vel[i] * dt +
                          0.5 * dt * dt * (acc_glob[i] + eskf->state.g[i]);
    eskf->state.vel[i] += dt * (acc_glob[i] + eskf->state.g[i]);
  }

  // update covariance
  // -R[am-ab]x dt
  vec_skew(a_x, acc_body);
  quat_to_rot_mat(Rq, eskf->state.quat);
  arm_mat_mult_f32(&R, &Ra, &Ra);
  arm_mat_scale_f32(&Ra, -dt, &Ra);

  // RT{(wm-wb)dt}
  quat_to_rot_mat(wrot, gyro_quat);
  arm_mat_trans_f32(&W, &W);

  // set F
  mat_set3x3(&F, 3, 6, a_x);
  mat_set3x3(&F, 3, 9, Rq);
  mat_set3x3(&F, 6, 6, wrot);
  mat_set_diag(&F, 0, 3, dt);
  mat_set_diag(&F, 6, 12, -dt);
  mat_scale3x3(&F, 3, 9, -dt);

  // set Q
  mat_set_diag(&Q, 3, 3, eskf->sigma_an * eskf->sigma_an * dt * dt);
  mat_set_diag(&Q, 6, 6, eskf->sigma_wn * eskf->sigma_wn * dt * dt);
  mat_set_diag(&Q, 9, 9, eskf->sigma_aw * eskf->sigma_aw * dt);
  mat_set_diag(&Q, 12, 12, eskf->sigma_ww * eskf->sigma_ww * dt);

  // P = F*P*F.T + Q
  arm_mat_trans_f32(&F, &Ft);
  arm_mat_mult_f32(&F, &P, &F);
  arm_mat_mult_f32(&F, &Ft, &P);
  arm_mat_add_f32(&P, &Q, &P);

  // update error state (mean is initialized to 0, so not necessary)
  // dx = Fx * dx
}

static inline void euler_to_quat(float q[4], float roll, float pitch,
                                 float yaw) {
  float cr = arm_cos_f32(roll * 0.5f);
  float sr = arm_sin_f32(roll * 0.5f);
  float cp = arm_cos_f32(pitch * 0.5f);
  float sp = arm_sin_f32(pitch * 0.5f);
  float cy = arm_cos_f32(yaw * 0.5f);
  float sy = arm_sin_f32(yaw * 0.5f);

  q[0] = cr * cp * cy + sr * sp * sy; // w
  q[1] = sr * cp * cy - cr * sp * sy; // x
  q[2] = cr * sp * cy + sr * cp * sy; // y
  q[3] = cr * cp * sy - sr * sp * cy; // z
}

void eskf_init(eskf_t *eskf, float sigma_an, float sigma_wn, float sigma_aw,
               float sigma_ww, gyro3d_t *gyro_init, mag3d_t *mag_init,
               accel3d_t *accel_init) {
  eskf_state_t st = {
      .pos = {0, 0, 0},
      .vel = {0, 0, 0},
      .quat = {1, 0, 0, 0},
      .acc_b = {0, 0, 0},
      .gyro_b = {gyro_init->gyro_x, gyro_init->gyro_y, gyro_init->gyro_z},
      .g = {0, 0, -9.81}};
  eskf->state = st;
  float roll_init, pitch_init, yaw_init;
  float quat_init[4];
  // float rot_init[3];
  float mag_v[3] = {mag_init->MagX, mag_init->MagY, mag_init->MagZ};
  roll_init = atan2f(accel_init->accel_y, accel_init->accel_z);
  pitch_init = atan2f(-accel_init->accel_x,
                      sqrtf(accel_init->accel_y * accel_init->accel_y +
                            accel_init->accel_z * accel_init->accel_z));
  // rot_init[0] = roll_init;
  // rot_init[1] = 0;
  // rot_init[2] = 0;
  // rot_to_quat(quat_init, rot_init);
  euler_to_quat(quat_init, roll_init, pitch_init, 0);
  quat_norm(quat_init);
  // quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
  // quat_norm(eskf->state.quat);

  // rot_init[0] = 0;
  // rot_init[1] = pitch_init;
  // rot_init[2] = 0;
  // rot_to_quat(quat_init, rot_init);
  // euler_to_quat(eskf->dx.quat, roll_init, pitch_init, 0);
  // quat_norm(quat_init);
  // quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
  // quat_norm(eskf->state.quat);

  // apply yaw from mag
  quat_rotate_vec(mag_v, quat_init, mag_v);
  yaw_init = atan2f(mag_v[1], mag_v[0]) + MAG_DECL;
  // rot_init[0] = 0;
  // rot_init[1] = 0;
  // rot_init[2] = yaw_init;
  // rot_to_quat(quat_init, rot_init);
  euler_to_quat(quat_init, roll_init, pitch_init, yaw_init);
  quat_norm(quat_init);
  quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
  quat_norm(eskf->state.quat);

  memset(eskf->P, 0, sizeof(eskf->P));
  memset(F_data, 0, sizeof(F_data));
  memset(Q_data, 0, sizeof(Q_data));
  eskf->sigma_an = sigma_an;
  eskf->sigma_aw = sigma_aw;
  eskf->sigma_wn = sigma_wn;
  eskf->sigma_ww = sigma_ww;

  // diagonal
  for (size_t i = 0; i < 15; i++) {
    F_data[i * 15 + i] = 1;
  }

  arm_mat_init_f32(&Ra, 3, 3, a_x);
  arm_mat_init_f32(&W, 3, 3, wrot);

  arm_mat_init_f32(&R, 3, 3, Rq);
  arm_mat_init_f32(&F, 15, 15, F_data);
  arm_mat_init_f32(&Ft, 15, 15, Ft_data);

  arm_mat_init_f32(&Q, 15, 15, Q_data);
  arm_mat_init_f32(&P, 15, 15, eskf->P);

  arm_mat_init_f32(&HPH, 3, 3, HPH_data);
  arm_mat_init_f32(&H, 3, 3, H_data);
  arm_mat_init_f32(&Ht, 3, 3, Ht_data);
  arm_mat_init_f32(&K, 3, 3, K_data);
  arm_mat_init_f32(&Pv, 3, 3, P_data);
}

void quat_get_euler(const float q[4], float *roll, float *pitch, float *yaw) {

  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  // --- Roll (x-axis rotation) ---
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  *roll = atan2f(sinr_cosp, cosr_cosp);

  // --- Pitch (y-axis rotation) ---
  float sinp = 2.0f * (w * y - z * x);
  if (fabsf(sinp) >= 1.0f)
    *pitch = copysignf(M_PI / 2.0f, sinp); // clamp for numerical safety
  else
    *pitch = asinf(sinp);

  // --- Yaw (z-axis rotation) ---
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  *yaw = atan2f(siny_cosp, cosy_cosp);
}

void quat_get_yaw(const float q[4], float *yaw) {

  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  // --- Yaw (z-axis rotation) ---
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  *yaw = atan2f(siny_cosp, cosy_cosp);
}

// Note: more general and efficient update for small angle error:
// `dx.quat = state.quat*-1 * yaw_quat`
void eskf_update_yaw(eskf_t *eskf, mag3d_t *mag, float cov) {

  float mag_v[3] = {mag->MagX, mag->MagY, mag->MagZ};
  float quat_rot[4];
  arm_status status;
  float state_yaw, state_roll, state_pitch;
  quat_get_euler(eskf->state.quat, &state_roll, &state_pitch, &state_yaw);
  euler_to_quat(quat_rot, state_roll, state_pitch, 0);
  quat_rotate_vec(mag_v, quat_rot, mag_v);
  float yaw = atan2f(mag_v[1], mag_v[0]) + MAG_DECL;

  mag_v[0] = 0;
  mag_v[1] = 0;
  mag_v[2] = state_yaw - yaw;
  mag_v[2] = mag_v[2] < M_PI ? mag_v[2] : mag_v[2] - 2.0f * M_PI;
  mag_v[2] = mag_v[2] > -M_PI ? mag_v[2] : mag_v[2] + 2.0f * M_PI;
  rot_to_quat(quat_rot, mag_v); // small angle difference
  // quat_to_vec(quat_v, eskf->state.quat);

  // compute kalman gain
  memset(H_data, 0, sizeof(H_data));
  H_data[8] = 1;

  // K = P*Ht*(H*P*Ht)^-1
  eskf_get_cov_quat(eskf, P_data);
  status = arm_mat_mult_f32(&H, &P, &HPH);
  status = arm_mat_trans_f32(&H, &Ht);
  status = arm_mat_mult_f32(&HPH, &Ht, &HPH);
  status = arm_mat_inverse_f32(&HPH, &H);
  status = arm_mat_mult_f32(&Ht, &H, &H);
  status = arm_mat_mult_f32(&K, &H, &K);

  // compute error state change
  // arm_mat_vec_mult_f32(&K, yaw_v, yaw_v);
  rot_to_quat(eskf->dx.quat, mag_v); // TODO

  // covariance update
  // symmetric form K(HPH.T+V)K.T
  // TODO: Joseph form
  arm_mat_trans_f32(&K, &Ht);
  arm_mat_mult_f32(&K, &HPH, &K);
  arm_mat_mult_f32(&K, &Ht, &K);

  // P <- P - K(HPH.T+V)K.T
  arm_mat_sub_f32(&Pv, &K, &Pv);
  set3x3(eskf->P, 6, 6, P_data, 15);

  // inject error-state
  quat_mul(eskf->state.quat, eskf->state.quat, eskf->dx.quat);
  quat_norm(eskf->state.quat);

  // reset error-state mean
  quat_reset(eskf->dx.quat);
}

void eskf_update_alt(eskf_t *eskf, float alt, float cov) {

  float alt_v[3] = {0, 0, alt - eskf->state.pos[2]};

  // compute kalman gain
  memset(H_data, 0, sizeof(H_data));
  H_data[8] = 1;

  // observation
  // K = P*Ht*(H*P*Ht)^-1
  eskf_get_cov_pos(eskf, P_data);
  arm_mat_mult_f32(&H, &P, &HPH);
  arm_mat_trans_f32(&H, &Ht);
  arm_mat_mult_f32(&HPH, &Ht, &HPH);
  arm_mat_inverse_f32(&HPH, &H);
  arm_mat_mult_f32(&Ht, &H, &H);
  arm_mat_mult_f32(&K, &H, &K);

  // compute error state change
  arm_mat_vec_mult_f32(&K, alt_v, eskf->dx.pos);

  // covariance update
  // symmetric form K(HPH.T+V)K.T
  // TODO: Joseph form
  arm_mat_trans_f32(&K, &Ht);
  arm_mat_mult_f32(&K, &HPH, &K);
  arm_mat_mult_f32(&K, &Ht, &K);

  // P <- P - K(HPH.T+V)K.T
  arm_mat_sub_f32(&Pv, &K, &Pv);
  set3x3(eskf->P, 6, 6, P_data, 15);

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

void eskf_get_cov_quat(eskf_t *eskf, float dst[9]) {
  dst[0] = eskf->P[15 * 6 + 6];
  dst[1] = eskf->P[15 * 6 + 7];
  dst[2] = eskf->P[15 * 6 + 8];
  dst[3] = eskf->P[15 * 7 + 6];
  dst[4] = eskf->P[15 * 7 + 7];
  dst[5] = eskf->P[15 * 7 + 8];
  dst[6] = eskf->P[15 * 8 + 6];
  dst[7] = eskf->P[15 * 8 + 7];
  dst[8] = eskf->P[15 * 8 + 8];
}

void eskf_get_cov_posvel(eskf_t *eskf, float dst[15]) {
  for (size_t i = 0; i < 15; i++) {
    dst[i] = eskf->P[(i / 6) * 15 + i % 6];
  }
}

void eskf_get_cov_pos(eskf_t *eskf, float dst[9]) {
  for (size_t i = 0; i < 9; i++) {
    dst[i] = eskf->P[(i / 3) * 15 + i % 3];
  }
}
