// Error-State Kalman Filter implementation optimized for ARM Cortex-M.
// Based on Sola et al. arXiv:1711.02508
//
// Author: Maks Boiar

#include "imu.h"
#include <math.h>
#include <memory.h>
#include <stdint.h>

#include "logger.h"
#include "quat_utils.h"

// float acc_noise = 1e-3f;  // m/s^2/sqrt(s)
// float gyro_noise = 1e-4f; // rad/s/sqrt(s)
// float acc_psd = 400;      // ug/sqrt(Hz)
// float gyro_rnsd = 0.005;  // deg/s/sqrt(Hz)

#define MAG_UPDATE_YAW 0
#define MAG_UPDATE_FULL 1
#define MAG_UPDATE_METHOD 0

#define m 1
#define m3 3

static inline void mat_set3x3(arm_matrix_instance_f32 *M, uint16_t r0,
                              uint16_t c0, const float mat[9]) {
  float *p = M->pData + r0 * M->numCols + c0;
  const uint16_t N = M->numCols;

  p[0] = mat[0];
  p[1] = mat[1];
  p[2] = mat[2];
  p[N + 0] = mat[3];
  p[N + 1] = mat[4];
  p[N + 2] = mat[5];
  p[2 * N + 0] = mat[6];
  p[2 * N + 1] = mat[7];
  p[2 * N + 2] = mat[8];
}

static inline void set3x3(float *dst, uint16_t r0, uint16_t c0,
                          const float mat[9], uint16_t ncols) {
  float *p = dst + r0 * ncols + c0;

  p[0] = mat[0];
  p[1] = mat[1];
  p[2] = mat[2];
  p[ncols + 0] = mat[3];
  p[ncols + 1] = mat[4];
  p[ncols + 2] = mat[5];
  p[2 * ncols + 0] = mat[6];
  p[2 * ncols + 1] = mat[7];
  p[2 * ncols + 2] = mat[8];
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
  float mod = rot[0] * rot[0] + rot[1] * rot[1] + rot[2] * rot[2];
  float theta = sqrtf(mod);
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
  float mod;
  arm_atan2_f32(1, src[0], &mod);
  dst[0] = src[1] * 2 * mod;
  dst[1] = src[2] * 2 * mod;
  dst[2] = src[3] * 2 * mod;
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
  float mod;
  arm_sqrt_f32(src[0] * src[0] + src[1] * src[1] + src[2] * src[2] +
                   src[3] * src[3],
               &mod);
  if (mod > 0) {
    src[0] = src[0] / mod;
    src[1] = src[1] / mod;
    src[2] = src[2] / mod;
    src[3] = src[3] / mod;
  }
}

static inline void vec_norm(float src[3]) {
  float mod;
  arm_sqrt_f32(src[0] * src[0] + src[1] * src[1] + src[2] * src[2], &mod);
  if (mod > 0) {
    src[0] = src[0] / mod;
    src[1] = src[1] / mod;
    src[2] = src[2] / mod;
  }
}

static inline void vec3_cross(float a[3], float b[3], float dst[3]) {
  dst[0] = a[1] * b[2] - a[2] * b[1];
  dst[1] = a[2] * b[0] - a[0] * b[2];
  dst[2] = a[0] * b[1] - a[1] * b[0];
}

static inline void vec_skew(float dst[9], const float src[3]) {
  dst[0] = 0;
  dst[1] = -src[2];
  dst[2] = src[1];
  dst[3] = src[2];
  dst[4] = 0;
  dst[5] = -src[0];
  dst[6] = -src[1];
  dst[7] = src[0];
  dst[8] = 0;
}

static inline void quat_from_two_vectors(float dst[4], float src1[3],
                                         float src2[3]) {
  vec_norm(src1);
  vec_norm(src2);
  float tmp[3];
  float d = src1[0] * src1[0] + src1[1] * src1[1] + src1[2] * src1[2];
  vec3_cross(src1, src2, tmp);
  d = sqrtf((1.0f + d) * 2.0f);
  dst[0] = 0.5f * d;
  d = 1.0f / d;
  dst[1] = tmp[0] * d;
  dst[2] = tmp[1] * d;
  dst[3] = tmp[2] * d;
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

static void quat_get_yaw(const float q[4], float *yaw) {

  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];

  // --- Yaw (z-axis rotation) ---
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  *yaw = atan2f(siny_cosp, cosy_cosp);
}

static arm_matrix_instance_f32 F, P, Ft, Q, Ra, R, W, HPH, K, H, Ht, V, HPH3,
    H3, Ht3, K3, V3, KP, HPHi, HPHi3, HPHic, HPHic3;
static float Ft_data[15 * 15], F_data[15 * 15], Q_data[15 * 15], V_data[m * m],
    V3_data[m3 * m3], KP_data[15 * 15], HPHi_data[m * m], HPHi3_data[m3 * m3],
    HPHic_data[m * m], HPHic3_data[m3 * m3];

static float H_data[m * 15], Ht_data[15 * m], K_data[15 * m], HPH_data[m * m];
static float H_data3[m3 * 15], Ht_data3[15 * m3], K_data3[15 * m3],
    HPH_data3[m3 * m3], P_data[15 * 15];
static float acc_glob[3];                    // acceleration in global frame
static float acc_body[3];                    // acceleration in body frame
static float gyro_body[3], gyro_body_int[3]; // angular velocity in body frame
static float gyro_quat[4];
static float a_x[9], Rq[9], wrot[9];
arm_status status;

int eskf_predict(eskf_t *eskf, const accel3d_t *acc_m, const gyro3d_t *gyro_m,
                 const float dt) {
  arm_status status;

  if (dt <= 0 || dt > 1) {
    LOG_ERR(0, "ESKF_INV_DT %f", dt);
    return 0;
  }
  // float quat_body_to_ned[4];

  // update nominal state

  acc_body[0] = acc_m->accel_y;
  acc_body[1] = acc_m->accel_x;
  acc_body[2] = acc_m->accel_z;
  gyro_body[0] = gyro_m->gyro_y; // ?
  gyro_body[1] = gyro_m->gyro_x;
  gyro_body[2] = gyro_m->gyro_z;

  linv3(acc_body, acc_body, eskf->state.acc_b, 9.81f, -1);
  linv3(gyro_body_int, gyro_body, eskf->state.gyro_b, dt,
        -dt); // substract bias
  // quat_inv(quat_body_to_ned, eskf->state.quat);
  // rot_to_quat(gyro_quat, gyro_body_int);
  euler_to_quat(gyro_quat, gyro_body_int[0], gyro_body_int[1],
                gyro_body_int[2]);
  quat_mul(eskf->state.quat, eskf->state.quat, gyro_quat);
  quat_norm(eskf->state.quat);
  quat_rotate_vec(acc_glob, eskf->state.quat, acc_body); // body -> world

  // integrate nominal kinematics
  for (int i = 0; i < 3; i++) {
    eskf->state.pos[i] += eskf->state.vel[i] * dt +
                          0.5 * dt * dt * (acc_glob[i] + eskf->state.g[i]);
    eskf->state.vel[i] += dt * (acc_glob[i] + eskf->state.g[i]);
  }

  if (!isfinite(eskf->state.quat[0])) {
    LOG_CRIT(0, "ESKF_QUAT_INF");
    return -2;
  }

  // update covariance
  // -R[am-ab]x dt
  vec_skew(a_x, acc_body);
  quat_to_rot_mat(Rq, eskf->state.quat);
  status = arm_mat_mult_f32(&R, &Ra, &Ra);
  status = arm_mat_scale_f32(&Ra, -dt, &Ra);
  status = arm_mat_scale_f32(&R, -dt, &R);

  // RT{(wm-wb)dt}
  quat_to_rot_mat(wrot, gyro_quat);
  status = arm_mat_trans_f32(&W, &W);

  // set F
  memset(F_data, 0, sizeof(F_data));
  mat_set3x3(&F, 3, 6, a_x);
  mat_set3x3(&F, 3, 9, Rq);
  mat_set3x3(&F, 6, 6, wrot);
  mat_set_diag(&F, 0, 3, dt);
  mat_set_diag(&F, 0, 0, 1);
  mat_set_diag(&F, 3, 3, 1);
  mat_set_diag(&F, 9, 9, 1);
  mat_set_diag(&F, 12, 12, 1);
  mat_set_diag(&F, 6, 12, -dt);

  // set Q
  memset(Q_data, 0, sizeof(Q_data));
  mat_set_diag(&Q, 3, 3, eskf->sigma_an * eskf->sigma_an * dt * dt);
  mat_set_diag(&Q, 6, 6, eskf->sigma_wn * eskf->sigma_wn * dt * dt);
  mat_set_diag(&Q, 9, 9, eskf->sigma_aw * eskf->sigma_aw * dt);
  mat_set_diag(&Q, 12, 12, eskf->sigma_ww * eskf->sigma_ww * dt);

  // P = F*P*F.T + Q
  status = arm_mat_trans_f32(&F, &Ft);
  status = arm_mat_mult_f32(&F, &P, &F);
  status = arm_mat_mult_f32(&F, &Ft, &P);
  status = arm_mat_add_f32(&P, &Q, &P);

  for (int i = 0; i < P.numCols; i++) {
    if (!isfinite(P_data[i * P.numCols + i])) {
      LOG_ERR(0, "ESKF_COV_NAN at %d", i * P.numCols + i);

      // Reset covariance to still be able to receive updates
      memset(P_data, 0, sizeof(P_data));
      return 0;
    }
    if (P_data[i * P.numCols + i] < 0) {
      P_data[i * P.numCols + i] = 0;
    }
  }

  return 0;

  // update error state (mean is initialized to 0, so not necessary)
  // dx = Fx * dx
}

void eskf_init(eskf_t *eskf, float sigma_an, float sigma_wn, float sigma_aw,
               float sigma_ww, gyro3d_t *gyro_init, mag3d_t *mag_init,
               accel3d_t *accel_init) {
  eskf_state_t st = {
      .pos = {0, 0, 0},
      .vel = {0, 0, 0},
      .quat = {1, 0, 0, 0},
      .acc_b = {0, 0, 0},
      .gyro_b = {gyro_init->gyro_y, gyro_init->gyro_x, gyro_init->gyro_z},
      .g = {0, 0, -9.81}};
  eskf->state = st;
  float roll_init, pitch_init, yaw_init;
  float quat_init[4];
  float acc_init[3] = {accel_init->accel_y, accel_init->accel_x,
                       accel_init->accel_z}; // local NED frame ??

  float mag_v[3] = {mag_init->MagY, mag_init->MagX,
                    -mag_init->MagZ}; // local NED frame
  roll_init = atan2f(acc_init[1], acc_init[2]);
  pitch_init = atan2f(-acc_init[0], sqrtf(acc_init[2] * acc_init[2] +
                                          acc_init[1] * acc_init[1]));
  euler_to_quat(quat_init, roll_init, pitch_init,
                0); // body->world, i.e. x'=Rx gives x' in global coordinates
  quat_norm(quat_init);
  // quat_inv(quat_init, quat_init);
  vec_norm(mag_v);

  // yaw version
  if (MAG_UPDATE_METHOD == MAG_UPDATE_YAW) {
    quat_rotate_vec(mag_v, quat_init, mag_v);
    yaw_init = atan2f(mag_v[1], mag_v[0]) + MAG_DECL;
    euler_to_quat(quat_init, roll_init, pitch_init, yaw_init);
    quat_norm(quat_init);
    quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
    quat_norm(eskf->state.quat);
  } else {
    // full mag field vector version

    float mag_pred[3] = {cosf(MAG_INCL) * cosf(MAG_DECL),
                         cosf(MAG_INCL) * sinf(MAG_DECL), sinf(MAG_INCL)};
    quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
    quat_inv(quat_init, quat_init);
    quat_rotate_vec(mag_pred, quat_init, mag_pred);
    // linv3(mag_diff, mag_pred, mag_v, 1, -1);
    quat_from_two_vectors(quat_init, mag_pred, mag_v);
    quat_norm(quat_init);
    quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
    quat_norm(eskf->state.quat);
  }

  memset(P_data, 0, sizeof(P_data));
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

  arm_mat_init_f32(&V, m, m, V_data);
  arm_mat_init_f32(&V, m3, m3, V_data);

  arm_mat_init_f32(&R, 3, 3, Rq);
  arm_mat_init_f32(&F, 15, 15, F_data);
  arm_mat_init_f32(&Ft, 15, 15, Ft_data);

  arm_mat_init_f32(&Q, 15, 15, Q_data);
  arm_mat_init_f32(&P, 15, 15, P_data);
  arm_mat_init_f32(&KP, 15, 15, KP_data);

  arm_mat_init_f32(&HPH, m, m, HPH_data);
  arm_mat_init_f32(&H, m, 15, H_data);
  arm_mat_init_f32(&Ht, 15, m, Ht_data);
  arm_mat_init_f32(&K, 15, m, K_data);

  arm_mat_init_f32(&HPH3, m3, m3, HPH_data3);
  arm_mat_init_f32(&H3, m3, 15, H_data3);
  arm_mat_init_f32(&Ht3, 15, m3, Ht_data3);
  arm_mat_init_f32(&K3, 15, m3, K_data3);

  arm_mat_init_f32(&HPHi3, m3, m3, HPHi3_data);
  arm_mat_init_f32(&HPHi, m, m, HPHi_data);
  arm_mat_init_f32(&HPHic, m, m, HPHic_data);
  arm_mat_init_f32(&HPHic3, m3, m3, HPHic3_data);
}

int eskf_update_yaw(eskf_t *eskf, mag3d_t *mag, float cov) {
  float mag_meas[3] = {mag->MagY, mag->MagX, -mag->MagZ};
  // Normalize magnetometer measurement: we only need orientation
  vec_norm(mag_meas);

  float quat_rot[4];
  float state_yaw, state_roll, state_pitch, yaw;

  if (MAG_UPDATE_METHOD == MAG_UPDATE_YAW) {
    quat_get_euler(eskf->state.quat, &state_roll, &state_pitch, &state_yaw);
    euler_to_quat(quat_rot, state_roll, state_pitch, 0);
    quat_rotate_vec(mag_meas, quat_rot, mag_meas);
    yaw = atan2f(mag_meas[1], mag_meas[0]) + MAG_DECL;

    eskf->dx[8] = state_yaw - yaw;
    memset(H_data, 0, sizeof(H_data));
    H_data[8] = 1;
  } else {
    // float mag_diff[3];
    // float quat_conj[4];
    // float mag_true_skew[3 * 3];
    // float q_data[3 * 4];
    // float mag_pred[3] = {cosf(MAG_INCL) * cosf(MAG_DECL),
    //                      cosf(MAG_INCL) * sinf(MAG_DECL), sinf(MAG_INCL)};

    // // H = dh/dq = dm/dtheta * dtheta/dq
    // quat_rotate_vec(mag_pred, eskf->state.quat, mag_pred);

    // linv3(mag_diff, mag_pred, mag_meas, 1, -1);
    // vec_skew(mag_true_skew, mag_pred);
    // arm_matrix_instance_f32 mag_pred_arm;
    // arm_mat_init_f32(&q_arm, 3, 4, &q_data);
    // q_data[0] = -eskf->state.quat[1];
    // q_data[1] = -eskf->state.quat[2];
    // q_data[2] = -eskf->state.quat[3];
    // q_data[3] = eskf->state.quat[0];
    // q_data[4] = -eskf->state.quat[3];
    // q_data[5] = eskf->state.quat[2];
    // q_data[6] = eskf->state.quat[3];
    // q_data[7] = eskf->state.quat[0];
    // q_data[8] = -eskf->state.quat[1];
    // q_data[9] = -eskf->state.quat[2];
    // q_data[10] = eskf->state.quat[1];
    // q_data[11] = eskf->state.quat[0];

    // arm_mat_init_f32(&mag_pred_arm, 3, 3, mag_true_skew);
    // status = arm_mat_scale_f32(&mag_pred_arm, -1, &mag_pred_arm);

    // in case of dh/ddtheta = dh/dq dq/ddtheta mag depends on theta directly
    // arm_mat_scale_f32(&q_arm, 1 / 2.0f, &q_arm);
    // arm_mat_mult_f32(&mag_pred_arm, &q_arm, &mag_pred)

    // euler_to_quat(quat_rot, 0, 0, yaw0);
    // quat_inv(quat_conj, eskf->state.quat);
    // quat_mul(quat_rot, quat_conj, quat_rot);
    // memcpy(H_data, mag_true_skew, sizeof(H_data));
    // arm_mat_vec_mult_f32(&K, mag_diff, eskf->dx.theta);
  }

  // measurement error covariance
  memset(V_data, 0, sizeof(V_data));
  V_data[0] = cov;

  // K = P*Ht*(H*P*Ht+V)^-1
  status = arm_mat_trans_f32(&H, &Ht);      // = 15*m
  status = arm_mat_mult_f32(&H, &P, &H);    // = m*15
  status = arm_mat_mult_f32(&H, &Ht, &HPH); // = m * m
  status = arm_mat_add_f32(&HPH, &V, &HPH); // m * m
  memcpy(HPHic_data, HPH_data, sizeof(HPH_data));
  status = arm_mat_inverse_f32(&HPHic, &HPHi); // = m * m

  if (status == ARM_MATH_SINGULAR) {
    LOG_ERR(0, "SINGULAR_MATRIX");
    return -1;
  }

  status = arm_mat_mult_f32(&Ht, &HPHi, &Ht); // = 15 * m
  status = arm_mat_mult_f32(&P, &Ht, &K);     // = 15 * m

  // compute error state change
  arm_mat_vec_mult_f32(&K, &eskf->dx[8], eskf->dx);

  // covariance update
  // symmetric form K(HPH.T+V)K.T
  // TODO: Joseph form
  arm_mat_trans_f32(&K, &H); // m * 15
  // status = arm_mat_add_f32(&HPH, &V, &HPH); // m * m
  arm_mat_mult_f32(&K, &HPH, &K); // 15 * m
  arm_mat_mult_f32(&K, &H, &KP);  // 15 * 15

  // P <- P - K(HPH.T+V)K.T
  arm_mat_sub_f32(&P, &KP, &P);

  for (int i = 0; i < P.numCols; i++) {
    if (!isfinite(P_data[i * P.numCols + i])) {
      LOG_CRIT(0, "ESKF_COV_INF at %d", i * P.numCols + i < 0);
      return -1;
    }
    if (P_data[i * P.numCols + i] < 0) {
      P_data[i * P.numCols + i] = 0;
    }
  }

  // inject error-state
  euler_to_quat(quat_rot, 0, 0, eskf->dx[8]);
  quat_mul(eskf->state.quat, eskf->state.quat, quat_rot);
  quat_norm(eskf->state.quat);

  // reset error-state mean
  memset(eskf->dx, 0, sizeof(eskf->dx));

  return 0;
}

void eskf_update_gps(eskf_t *eskf, const GPS_data *data, uint64_t home_lon,
                     uint64_t home_lat, float home_alt) {
  float pos_diff[3];
  lla_to_ned(data->lat, data->lon, data->alt, home_lat, home_lon, home_alt,
             pos_diff);

  // compute kalman gain
  memset(H_data3, 0, sizeof(H_data));
  H_data3[0] = 1;
  H_data3[m * 1 + 1] = 1;
  H_data3[m * 2 + 2] = 1;

  // measurement error covariance
  memset(V3_data, 0, sizeof(V3_data));
  V3_data[m3 * 0 + 2] = data->hdop;
  V3_data[m3 * 1 + 2] = data->hdop;
  V3_data[m3 * 2 + 2] = data->vdop;

  // K = P*Ht*(H*P*Ht+V)^-1
  status = arm_mat_trans_f32(&H3, &Ht3);       // = 15*m
  status = arm_mat_mult_f32(&H3, &P, &H3);     // = m*15
  status = arm_mat_mult_f32(&H3, &Ht3, &HPH3); // = m * m
  status = arm_mat_add_f32(&HPH3, &V3, &HPH3); // m * m
  memcpy(HPHic3_data, HPH_data3, sizeof(HPH_data3));
  status = arm_mat_inverse_f32(&HPHic3, &HPHi3); // = m * m
  if (status == ARM_MATH_SINGULAR) {
    // TODO: log
    return;
  }
  status = arm_mat_mult_f32(&Ht3, &HPHi3, &Ht3); // = 15 * m
  status = arm_mat_mult_f32(&P, &Ht3, &K3);      // = 15 * m

  // compute error state change
  arm_mat_vec_mult_f32(&K3, pos_diff, eskf->dx);

  // covariance update
  // symmetric form K(HPH.T+V)K.T
  // TODO: Joseph form
  arm_mat_trans_f32(&K3, &H3);
  // status = arm_mat_add_f32(&HPH3, &V3, &HPH3);
  arm_mat_mult_f32(&K3, &HPH3, &K3);
  arm_mat_mult_f32(&K3, &H3, &KP);

  // P <- P - K(HPH.T+V)K.T
  arm_mat_sub_f32(&P, &KP, &P);
  // set3x3(P_data, 6, 6, P_data, 15);

  // inject error-state
  linv3(eskf->state.pos, eskf->state.pos, eskf->dx, 1, 1);

  // reset error-state mean
  memset(&eskf->dx, 0, sizeof(eskf->dx));
}

int eskf_update_baro(eskf_t *eskf, float alt, float cov) {
  float alt_v[1] = {alt - eskf->state.pos[2]};

  // compute kalman gain
  memset(H_data, 0, sizeof(H_data));
  H_data[2] = 1; // updates z component of position

  // measurement error covariance
  memset(V_data, 0, sizeof(V_data));
  V_data[0] = cov;

  // K = P*Ht*(H*P*Ht+V)^-1
  status = arm_mat_trans_f32(&H, &Ht);      // = 15*m
  status = arm_mat_mult_f32(&H, &P, &H);    // = m*15
  status = arm_mat_mult_f32(&H, &Ht, &HPH); // = m * m
  status = arm_mat_add_f32(&HPH, &V, &HPH);
  memcpy(HPHic_data, HPH_data, sizeof(HPH_data));
  status = arm_mat_inverse_f32(&HPHic, &HPHi); // = m * m
  if (status == ARM_MATH_SINGULAR) {
    LOG_ERR(0, "SINGULAR_MATRIX");
    return -1;
  }
  status = arm_mat_mult_f32(&Ht, &HPHi, &Ht); // = 15 * m
  status = arm_mat_mult_f32(&P, &Ht, &K);     // = 15 * m

  // compute error state change
  arm_mat_vec_mult_f32(&K, alt_v, eskf->dx);

  // covariance update
  // symmetric form K(HPH.T+V)K.T
  // TODO: Joseph form
  status = arm_mat_trans_f32(&K, &H);
  status = arm_mat_mult_f32(&K, &HPH, &K);
  status = arm_mat_mult_f32(&K, &H, &KP);

  // P <- P - K(HPH.T+V)K.T
  status = arm_mat_sub_f32(&P, &KP, &P);

  for (int i = 0; i < P.numCols; i++) {
    if (!isfinite(P_data[i * P.numCols + i])) {
      LOG_CRIT(0, "ESKF_COV_INF at %d", i * P.numCols + i < 0);
      return -1;
    }
    if (P_data[i * P.numCols + i] < 0) {
      P_data[i * P.numCols + i] = 0;
    }
  }

  // inject error-state
  linv3(eskf->state.pos, eskf->state.pos, eskf->dx, 1, 1);

  // reset error-state mean
  memset(&eskf->dx, 0, sizeof(eskf->dx));

  return 0;
}

void eskf_update_dist_sensor(eskf_t *eskf, float alt, float cov) {
  // float alt_v[1] = {alt - eskf->state.pos[2]};

  // // compute kalman gain
  // memset(H_data, 0, sizeof(H_data));
  // H_data[2] = 1; // updates z component of position

  // // K = P*Ht*(H*P*Ht)^-1
  // status = arm_mat_trans_f32(&H, &Ht);        // = 15*m
  // status = arm_mat_mult_f32(&H, &P, &H);      // = m*15
  // status = arm_mat_mult_f32(&H, &Ht, &HPH);   // = m * m
  // status = arm_mat_inverse_f32(&HPH, &HPHi);  // = m * m
  // status = arm_mat_mult_f32(&Ht, &HPHi, &Ht); // = 15 * m
  // status = arm_mat_mult_f32(&P, &Ht, &K);     // = 15 * m

  // // compute error state change
  // arm_mat_vec_mult_f32(&K, alt_v, eskf->dx.pos);

  // // measurement error covariance
  // arm_status status;
  // memset(V_data, 0, sizeof(V_data));
  // V_data[0] = cov;

  // // covariance update
  // // symmetric form K(HPH.T+V)K.T
  // // TODO: Joseph form
  // arm_mat_trans_f32(&K, &H);
  // status = arm_mat_add_f32(&HPH, &V, &HPH);
  // arm_mat_mult_f32(&K, &HPH, &K);
  // arm_mat_mult_f32(&K, &H, &KP);

  // // P <- P - K(HPH.T+V)K.T
  // arm_mat_sub_f32(&P, &KP, &P);
  // // set3x3(P_data, 6, 6, P_data, 15);

  // // inject error-state
  // linv3(eskf->state.pos, eskf->state.pos, eskf->dx.pos, 1, 1);

  // // reset error-state mean
  // memset(&eskf->dx.pos, 0, sizeof(eskf->dx.pos));
}

void eskf_reset(eskf_t *eskf) { memset(&eskf->dx, 0, sizeof(eskf->dx)); }

void eskf_inject(eskf_t *eskf, const eskf_state_t *es) {
  linv3(eskf->state.pos, eskf->state.pos, es->pos, 1, 1);
  linv3(eskf->state.vel, eskf->state.vel, es->vel, 1, 1);
  quat_mul(eskf->state.quat, eskf->state.quat, es->quat);
  linv3(eskf->state.acc_b, eskf->state.acc_b, es->acc_b, 1, 1);
  linv3(eskf->state.gyro_b, eskf->state.gyro_b, es->gyro_b, 1, 1);
}

void eskf_get_cov_orientation(eskf_t *eskf, float dst[9]) {
  dst[0] = P_data[15 * 6 + 6];
  dst[1] = P_data[15 * 6 + 7];
  dst[2] = P_data[15 * 6 + 8];
  dst[3] = P_data[15 * 7 + 6];
  dst[4] = P_data[15 * 7 + 7];
  dst[5] = P_data[15 * 7 + 8];
  dst[6] = P_data[15 * 8 + 6];
  dst[7] = P_data[15 * 8 + 7];
  dst[8] = P_data[15 * 8 + 8];
}

void eskf_get_cov_posvel(eskf_t *eskf, float dst[21]) {
  for (size_t i = 0; i < 21; i++) {
    dst[i] = P_data[(i / 6) * 15 + i % 6];
  }
}

void eskf_get_cov_pos(eskf_t *eskf, float dst[9]) {
  for (size_t i = 0; i < 9; i++) {
    dst[i] = P_data[(i / 3) * 15 + i % 3];
  }
}
