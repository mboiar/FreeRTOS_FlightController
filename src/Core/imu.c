// Error-State Kalman Filter implementation optimized for ARM Cortex-M.
// Based on Sola et al. arXiv:1711.02508
//
// Author: Maks Boiar

#include "imu.h"
#include <math.h>
#include <memory.h>
#include <stdint.h>

#include "Config.h"
#include "logger.h"
#include "quat_utils.h"

#define MAG_UPDATE_YAW 0
#define MAG_UPDATE_FULL 1
#define MAG_UPDATE_METHOD 0

#define m 1
#define m3 3
#define ESKFN 15

// static arm_matrix_instance_f32 F, P, Ft, Q, Ra, R, W, HPH, K, H, Ht, V, HPH3,
//     H3, Ht3, K3, V3, KP, HPHi, HPHi3, HPHic, HPHic3, HP, HP3;
// static float Ft_data[ESKFN * ESKFN], F_data[ESKFN * ESKFN],
//     Q_data[ESKFN * ESKFN], V_data[m * m], V3_data[m3 * m3],
//     KP_data[ESKFN * ESKFN], HPHi_data[m * m], HPHi3_data[m3 * m3],
//     HPHic_data[m * m], HPHic3_data[m3 * m3];

// static float H_data[m * ESKFN], HP_data[m * ESKFN], Ht_data[ESKFN * m],
//     K_data[ESKFN * m], HPH_data[m * m];
// static float H_data3[m3 * ESKFN], Ht_data3[ESKFN * m3], K_data3[ESKFN * m3],
//     HPH_data3[m3 * m3], HP3_data[m3 * ESKFN], P_data[ESKFN * ESKFN];
float acc_glob[3];                   // acceleration in global frame
static float acc_body[3], g_body[3]; // acceleration in body frame
static float gyro_body[3], gi[3];    // angular velocity in body frame
static float gyro_quat[4], quat_inv_[4];
// static float a_x[9], Rq[9], wrot[9];
arm_status status;

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

  // Avoid division by zero for very small rotations
  if (theta < 1e-6f) {
    // Small angle approximation: q ≈ [1, rot[0]/2, rot[1]/2, rot[2]/2]
    dst[0] = 1.0f;
    dst[1] = rot[0] * 0.5f;
    dst[2] = rot[1] * 0.5f;
    dst[3] = rot[2] * 0.5f;
  } else {
    // General case: q = [cos(theta/2), sin(theta/2) * normalized_axis]
    float sin_half = arm_sin_f32(theta / 2.0f);
    float scale = sin_half / theta;

    dst[0] = arm_cos_f32(theta / 2.0f);
    dst[1] = scale * rot[0];
    dst[2] = scale * rot[1];
    dst[3] = scale * rot[2];
  }
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
  float t0, t1, t2;
  t0 = a[1] * b[2] - a[2] * b[1];
  t1 = a[2] * b[0] - a[0] * b[2];
  t2 = a[0] * b[1] - a[1] * b[0];
  dst[0] = t0;
  dst[1] = t1;
  dst[2] = t2;
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
    *pitch = copysignf((float)M_PI / 2.0f, sinp); // clamp for numerical safety
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

int mahony_predict(eskf_t *eskf, const gyro3d_t *gyro_m, const float dt) {

  gyro_body[0] = gyro_m->gyro_y;
  gyro_body[1] = gyro_m->gyro_x;
  gyro_body[2] = gyro_m->gyro_z;

  // linv3(gyro_body, gyro_body, eskf->state.acc_b, 1,
  //       1); // correction
  linv3(gi, gyro_body, eskf->state.gyro_b, 1,
        -1); // substract bias

  // Scale angular velocity by time step to get rotation vector
  float rot_vec[3] = {gi[0] * dt, gi[1] * dt, gi[2] * dt};

  rot_to_quat(gyro_quat, rot_vec);

  // float q0, q1, q2, q3;
  // q0 = eskf->state.quat[0];
  // q1 = eskf->state.quat[1];
  // q2 = eskf->state.quat[2];
  // q3 = eskf->state.quat[3];
  // eskf->state.quat[0] += (-q1 * gi[0] - q2 * gi[1] - q3 * gi[2]);
  // eskf->state.quat[1] += (q0 * gi[0] - q2 * gi[2] - q3 * gi[1]);
  // eskf->state.quat[2] += (q0 * gi[1] - q1 * gi[2] - q3 * gi[0]);
  // eskf->state.quat[3] += (q0 * gi[2] - q1 * gi[1] - q2 * gi[0]);
  quat_norm(gyro_quat);

  quat_mul(eskf->state.quat, eskf->state.quat, gyro_quat);
  quat_norm(eskf->state.quat);

  return 0;
}

int mahony_update(eskf_t *eskf, const float acc_m[3], mag3d_t *mag,
                  float *yaw_m, float offv[3], float offM[3][3], float dt) {

  float mag_v[3] = {mag->MagY, mag->MagX, -mag->MagZ};
  float e[3] = {0, 0, 0};
  float h[3], pred_down[3] = {0, 0, 1}, pred_east[3] = {0, 1, 0};
  float wrot[9];

  // Normalize magnetometer measurement: we only need orientation

  float quat_rot[4];
  float state_yaw, state_roll, state_pitch, yaw;
  static float eInt[3] = {0.0, 0.0, 0.0};

  memset(eskf->state.acc_b, 0, sizeof(eskf->state.acc_b));

  acc_body[0] = acc_m[0];
  acc_body[1] = acc_m[1];
  acc_body[2] = acc_m[2];

  float acc_abs = sqrtf(acc_body[0] * acc_body[0] + acc_body[1] * acc_body[1] +
                        acc_body[2] * acc_body[2]);
  float vscale = 1.0f;

  // if significant acceleration not reliable
  if (acc_abs > 2.0f) {
    LOG_WARN(0, "ACC_UPD_SKIP");
    return 0;
  } else if (acc_abs > 1.1f) {
    vscale = 0.1f * MAHONY_KP;
    return 0;
  }

  // linv3(acc_body, acc_body, eskf->state.acc_b, 9.81f, -1);
  // quat_rotate_vec(acc_glob, eskf->state.quat,
  //                 acc_body); // body -> world

  quat_inv(quat_inv_, eskf->state.quat);
  quat_norm(quat_inv_);
  // quat_rotate_vec(g_body, quat_inv_, eskf->state.g);
  // vec3_cross(acc_body, g_body, acc_glob);

  // linv3(e, e, acc_glob, 1, 1);
  // linv3(acc_body, acc_body, g_body, 1, -1);

  // quat_get_euler(eskf->state.quat, &state_roll, &state_pitch, &state_yaw);
  // euler_to_quat(quat_rot, state_roll, state_pitch, 0.0);

  mag_v[0] -= offv[1];
  mag_v[1] -= offv[0];
  mag_v[2] -= -offv[2];

  mag_v[0] =
      offM[1][0] * mag_v[1] + offM[1][1] * mag_v[0] + offM[1][2] * -mag_v[2];
  mag_v[1] =
      offM[0][0] * mag_v[1] + offM[0][1] * mag_v[0] + offM[0][2] * -mag_v[2];
  mag_v[2] =
      offM[2][0] * mag_v[0] + offM[2][1] * mag_v[1] + offM[2][2] * -mag_v[2];

  vec_norm(mag_v);

  vec3_cross(acc_body, mag_v, h);
  vec_norm(h);

  quat_rotate_vec(pred_down, quat_inv_, pred_down);
  quat_rotate_vec(pred_east, quat_inv_, pred_east);
  vec3_cross(acc_body, pred_down, pred_down);
  vec3_cross(h, pred_east, pred_east);
  linv3(e, pred_down, pred_east, 1.0f, 0.0f);

  // quat_rotate_vec(mag_v, quat_rot, mag_v);
  // yaw = -atan2f(mag_v[1], mag_v[0]) + MAG_DECL;

  // if (yaw - state_yaw < -M_PI) {
  //   eskf->dx[8] = yaw - state_yaw + 2 * M_PI;
  // } else if (yaw - state_yaw > M_PI) {
  //   eskf->dx[8] = yaw - state_yaw - 2 * M_PI;
  // } else {
  //   eskf->dx[8] = yaw - state_yaw; // atan2f(sinf(yaw - state_yaw),
  //                                  // cosf(yaw - state_yaw));
  // }
  // *yaw_m = yaw;
  // float mag_diff[3];
  // float quat_conj[4];
  // float mag_true_skew[3 * 3];
  // float q_data[3 * 4];
  // float mag_pred[3] = {cosf(MAG_INCL) * cosf(MAG_DECL),
  //                      cosf(MAG_INCL) * sinf(MAG_DECL), sinf(MAG_INCL)};

  // // H = dh/dq = dm/dtheta * dtheta/dq
  // quat_rotate_vec(mag_pred, eskf->state.quat, mag_pred);

  // linv3(mag_diff, mag_pred, mag_meas, 1, -1);

  if (MAHONY_KI > 0.0f) {
    eInt[0] += e[0]; // accumulate integral error
    eInt[1] += e[1];
    eInt[2] += e[2];
    // Apply I feedback
    eskf->state.acc_b[0] += MAHONY_KI * eInt[0];
    eskf->state.acc_b[1] += MAHONY_KI * eInt[1];
    eskf->state.acc_b[2] += MAHONY_KI * eInt[2];
  }

  eskf->state.acc_b[0] += vscale * e[0];
  eskf->state.acc_b[1] += vscale * e[1];
  eskf->state.acc_b[2] += vscale * e[2];

  return 0;
}

void mahony_init(eskf_t *eskf, gyro3d_t *gyro_init, mag3d_t *mag_init,
                 accel3d_t *accel_init, float offv[3], float offM[3][3]) {
  eskf_state_t st = {
      .pos = {0, 0, 0},
      .vel = {0, 0, 0},
      .quat = {1, 0, 0, 0},
      .acc_b = {0, 0, 0},
      .gyro_b = {gyro_init->gyro_y, gyro_init->gyro_x, gyro_init->gyro_z},
      .g = {0, 0, 1}};
  eskf->state = st;
  float roll_init, pitch_init, yaw_init;
  float quat_init[4];
  float acc_init[3] = {accel_init->accel_y, accel_init->accel_x,
                       accel_init->accel_z}; // local

  float mag_v[3] = {mag_init->MagY, mag_init->MagX,
                    -mag_init->MagZ}; // local frame

  roll_init = atan2f(acc_init[1], acc_init[2]);
  pitch_init = atan2f(-acc_init[0], sqrtf(acc_init[2] * acc_init[2] +
                                          acc_init[1] * acc_init[1]));
  euler_to_quat(quat_init, roll_init, pitch_init,
                0); // body->world, i.e. x'=Rx gives x' in global coordinates
  quat_norm(quat_init);
  // quat_inv(quat_init, quat_init);
  // vec_norm(mag_v);

  mag_v[0] -= offv[1];
  mag_v[1] -= offv[0];
  mag_v[2] -= -offv[2];

  mag_v[0] =
      offM[1][0] * mag_v[1] + offM[1][1] * mag_v[0] + offM[1][2] * -mag_v[2];
  mag_v[1] =
      offM[0][0] * mag_v[1] + offM[0][1] * mag_v[0] + offM[0][2] * -mag_v[2];
  mag_v[2] =
      offM[2][0] * mag_v[0] + offM[2][1] * mag_v[1] + offM[2][2] * -mag_v[2];

  quat_rotate_vec(mag_v, quat_init, mag_v);

  yaw_init = -atan2f(mag_v[1], mag_v[0]) + MAG_DECL;
  euler_to_quat(quat_init, roll_init, pitch_init, yaw_init);
  quat_norm(quat_init);
  quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
  quat_norm(eskf->state.quat);
}

// int eskf_predict(eskf_t *eskf, const accel3d_t *acc_m, const gyro3d_t
// *gyro_m,
//                  const float dt) {
//   arm_status status = ARM_MATH_SUCCESS;

//   if (dt <= 0 || dt > 1) {
//     LOG_ERR(0, "ESKF_INV_DT %f", dt);
//     return 0;
//   }
//   // float quat_body_to_ned[4];

//   // update nominal state

//   acc_body[0] = acc_m->accel_y;
//   acc_body[1] = acc_m->accel_x;
//   acc_body[2] = acc_m->accel_z;
//   gyro_body[0] = gyro_m->gyro_y; // ?
//   gyro_body[1] = gyro_m->gyro_x;
//   gyro_body[2] = gyro_m->gyro_z;

//   linv3(acc_body, acc_body, eskf->state.acc_b, 9.81f, -1);
//   linv3(gyro_body_int, gyro_body, eskf->state.gyro_b, dt,
//         -dt); // substract bias
//   euler_to_quat(gyro_quat, gyro_body_int[0], gyro_body_int[1],
//                 gyro_body_int[2]);
//   // 0);
//   quat_norm(gyro_quat);
//   quat_mul(eskf->state.quat, eskf->state.quat, gyro_quat);
//   quat_norm(eskf->state.quat);
//   quat_rotate_vec(acc_glob, eskf->state.quat,
//                   acc_body); // body -> world

//   // integrate nominal kinematics
//   // for (int i = 0; i < 3; i++) {
//   //   eskf->state.pos[i] += eskf->state.vel[i] * dt +
//   //                         0.5 * dt * dt * (acc_glob[i] +
//   eskf->state.g[i]);
//   //   eskf->state.vel[i] += dt * (acc_glob[i] + eskf->state.g[i]);
//   // }

//   if (!isfinite(eskf->state.quat[0])) {
//     LOG_CRIT(0, "ESKF_QUAT_INF");
//     return -2;
//   }

//   // update covariance
//   // -R[am-ab]x dt   ??? - g???
//   quat_inv(quat_inv_, eskf->state.quat);
//   quat_rotate_vec(g_body, quat_inv_, eskf->state.g);
//   linv3(acc_body, acc_body, g_body, 1, 1);
//   // vec_skew(a_x, acc_body);
//   // quat_to_rot_mat(Rq, eskf->state.quat);

//   // status = arm_mat_mult_f32(&R, &Ra, &Ra);
//   // status = arm_mat_scale_f32(&Ra, -dt, &Ra);
//   // status = arm_mat_scale_f32(&R, -dt, &R);

//   // RT{(wm-wb)dt}
//   quat_to_rot_mat(wrot, gyro_quat);
//   status = arm_mat_trans_f32(&W, &W);

//   // set F
//   memset(F_data, 0, sizeof(F_data));
//   // mat_set3x3(&F, 3, 6, a_x);
//   // mat_set3x3(&F, 3, 9, Rq);
//   mat_set3x3(&F, 6, 6, wrot);
//   // mat_set_diag(&F, 0, 3, dt);
//   // mat_set_diag(&F, 0, 0, 1);
//   // mat_set_diag(&F, 3, 3, 1);
//   mat_set_diag(&F, 9, 9, 1);
//   mat_set_diag(&F, 12, 12, 1);
//   mat_set_diag(&F, 6, 12, -dt);

//   // mat_set_diag(&F, 15, 15, 1);
//   // mat_set_diag(&F, 3, 15, dt);

//   // set Q
//   memset(Q_data, 0, sizeof(Q_data));
//   // mat_set_diag(&Q, 3, 3, eskf->sigma_an * eskf->sigma_an * dt * dt);
//   mat_set_diag(&Q, 6, 6, eskf->sigma_wn * eskf->sigma_wn * dt * dt);
//   // mat_set_diag(&Q, 9, 9, eskf->sigma_aw * eskf->sigma_aw * dt);
//   mat_set_diag(&Q, 12, 12, eskf->sigma_ww * eskf->sigma_ww * dt);

//   // ???
//   // mat_set_diag(&Q, 0, 0, eskf->sigma_an * eskf->sigma_an * dt * dt * dt
//   // / 3.0f); mat_set_diag(&Q, 0, 3, eskf->sigma_an * eskf->sigma_an * dt *
//   dt
//   // / 2.0f); mat_set_diag(&Q, 3, 0, eskf->sigma_an * eskf->sigma_an * dt *
//   dt
//   // / 2.0f);

//   // P = F*P*F.T + Q
//   status = arm_mat_trans_f32(&F, &Ft);
//   status = arm_mat_mult_f32(&F, &P, &F);
//   status = arm_mat_mult_f32(&F, &Ft, &P);
//   status = arm_mat_add_f32(&P, &Q, &P);

//   for (int i = 0; i < P.numCols; i++) {
//     if (!isfinite(P_data[i * P.numCols + i])) {
//       LOG_ERR(0, "ESKF_COV_NAN at %d", i * P.numCols + i);

//       // Edgecase: reset position covariance to avoid explosion
//       // memset(P_data, 0, sizeof(float) * 15 * 3);
//       // memset(P_data + 3 * 15, 0, sizeof(float) * 15 * 3);
//       // for (int i = 3; i < 15; i++) {
//       //   memset(P_data + i * 15, 0, sizeof(float) * 3);
//       // }
//       // memset(P_data, 0, sizeof(P_data));

//       // for (int i = 6; i < 15; i++) {
//       //   memset(P_data + i * 15, 0, sizeof(float) * 15);
//       // }
//       return -1;
//     }
//     if (P_data[i * P.numCols + i] < 0) {
//       LOG_ERR(0, "ESKF_COV_SING at %d", i * P.numCols + i);
//       return -1;
//       // P_data[i * P.numCols + i] = 0;
//     }
//   }

//   return 0;

//   // update error state (mean is initialized to 0, so not necessary)
//   // dx = Fx * dx
// }

// void eskf_init(eskf_t *eskf, float sigma_an, float sigma_wn, float sigma_aw,
//                float sigma_ww, gyro3d_t *gyro_init, mag3d_t *mag_init,
//                accel3d_t *accel_init, float offv[3], float offM[3][3]) {
//   eskf_state_t st = {
//       .pos = {0, 0, 0},
//       .vel = {0, 0, 0},
//       .quat = {1, 0, 0, 0},
//       .acc_b = {0, 0, 0},
//       .gyro_b = {gyro_init->gyro_y, gyro_init->gyro_x, gyro_init->gyro_z},
//       .g = {0, 0, 9.81}};
//   eskf->state = st;
//   float roll_init, pitch_init, yaw_init;
//   float quat_init[4];
//   float acc_init[3] = {accel_init->accel_y, accel_init->accel_x,
//                        accel_init->accel_z}; // local

//   float mag_v[3] = {mag_init->MagY, mag_init->MagX,
//                     -mag_init->MagZ}; // local frame
//   roll_init = atan2f(acc_init[1], acc_init[2]);
//   pitch_init = atan2f(-acc_init[0], sqrtf(acc_init[2] * acc_init[2] +
//                                           acc_init[1] * acc_init[1]));
//   euler_to_quat(quat_init, roll_init, pitch_init,
//                 0); // body->world, i.e. x'=Rx gives x' in global coordinates
//   quat_norm(quat_init);
//   // quat_inv(quat_init, quat_init);
//   // vec_norm(mag_v);

//   // yaw version
//   if (MAG_UPDATE_METHOD == MAG_UPDATE_YAW) {

//     mag_v[0] -= offv[1];
//     mag_v[1] -= offv[0];
//     mag_v[2] -= -offv[2];

//     mag_v[0] =
//         offM[1][0] * mag_v[1] + offM[1][1] * mag_v[0] + offM[1][2] *
//         -mag_v[2];
//     mag_v[1] =
//         offM[0][0] * mag_v[1] + offM[0][1] * mag_v[0] + offM[0][2] *
//         -mag_v[2];
//     mag_v[2] =
//         offM[2][0] * mag_v[0] + offM[2][1] * mag_v[1] + offM[2][2] *
//         -mag_v[2];

//     quat_rotate_vec(mag_v, quat_init, mag_v);

//     yaw_init = -atan2f(mag_v[1], mag_v[0]) + MAG_DECL;
//     euler_to_quat(quat_init, roll_init, pitch_init, yaw_init);
//     quat_norm(quat_init);
//     quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
//     quat_norm(eskf->state.quat);
//   } else {
//     // full mag field vector version

//     float mag_pred[3] = {cosf(MAG_INCL) * cosf(MAG_DECL),
//                          cosf(MAG_INCL) * sinf(MAG_DECL), sinf(MAG_INCL)};
//     quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
//     quat_inv(quat_init, quat_init);
//     quat_rotate_vec(mag_pred, quat_init, mag_pred);
//     // linv3(mag_diff, mag_pred, mag_v, 1, -1);
//     quat_from_two_vectors(quat_init, mag_pred, mag_v);
//     quat_norm(quat_init);
//     quat_mul(eskf->state.quat, eskf->state.quat, quat_init);
//     quat_norm(eskf->state.quat);
//   }

//   memset(P_data, 0, sizeof(P_data));
//   memset(F_data, 0, sizeof(F_data));
//   memset(Q_data, 0, sizeof(Q_data));
//   eskf->sigma_an = sigma_an;
//   eskf->sigma_aw = sigma_aw;
//   eskf->sigma_wn = sigma_wn;
//   eskf->sigma_ww = sigma_ww;

//   // diagonal
//   for (size_t i = 0; i < ESKFN; i++) {
//     F_data[i * ESKFN + i] = 1;
//   }

//   for (size_t i = 0; i < ESKFN; i++) {
//     P_data[i * ESKFN + i] = 1e-7;
//   }

//   arm_mat_init_f32(&Ra, 3, 3, a_x);
//   arm_mat_init_f32(&W, 3, 3, wrot);

//   arm_mat_init_f32(&V, m, m, V_data);
//   arm_mat_init_f32(&V3, m3, m3, V3_data);

//   arm_mat_init_f32(&R, 3, 3, Rq);
//   arm_mat_init_f32(&F, ESKFN, ESKFN, F_data);
//   arm_mat_init_f32(&Ft, ESKFN, ESKFN, Ft_data);

//   arm_mat_init_f32(&Q, ESKFN, ESKFN, Q_data);
//   arm_mat_init_f32(&P, ESKFN, ESKFN, P_data);
//   arm_mat_init_f32(&KP, ESKFN, ESKFN, KP_data);

//   arm_mat_init_f32(&HPH, m, m, HPH_data);
//   arm_mat_init_f32(&H, m, ESKFN, H_data);
//   arm_mat_init_f32(&HP, m, ESKFN, HP_data);
//   arm_mat_init_f32(&Ht, ESKFN, m, Ht_data);
//   arm_mat_init_f32(&K, ESKFN, m, K_data);

//   arm_mat_init_f32(&HPH3, m3, m3, HPH_data3);
//   arm_mat_init_f32(&H3, m3, ESKFN, H_data3);
//   arm_mat_init_f32(&HP3, m3, ESKFN, HP3_data);
//   arm_mat_init_f32(&Ht3, ESKFN, m3, Ht_data3);
//   arm_mat_init_f32(&K3, ESKFN, m3, K_data3);

//   arm_mat_init_f32(&HPHi3, m3, m3, HPHi3_data);
//   arm_mat_init_f32(&HPHi, m, m, HPHi_data);
//   arm_mat_init_f32(&HPHic, m, m, HPHic_data);
//   arm_mat_init_f32(&HPHic3, m3, m3, HPHic3_data);
// }

// static void inject_error(eskf_t *eskf) {

//   // inject error-state
//   float quat_rot[4];
//   eskf->state.pos[0] += eskf->dx[0];
//   eskf->state.pos[1] += eskf->dx[1];
//   eskf->state.pos[2] += eskf->dx[2];
//   eskf->state.vel[0] += eskf->dx[3];
//   eskf->state.vel[1] += eskf->dx[4];
//   eskf->state.vel[2] += eskf->dx[5];
//   euler_to_quat(quat_rot, eskf->dx[6], eskf->dx[7], eskf->dx[8]);
//   quat_norm(quat_rot);

//   quat_mul(eskf->state.quat, eskf->state.quat, quat_rot);
//   quat_norm(eskf->state.quat);
//   eskf->state.acc_b[0] += eskf->dx[9];
//   eskf->state.acc_b[1] += eskf->dx[10];
//   eskf->state.acc_b[2] += eskf->dx[11];
//   eskf->state.gyro_b[0] += eskf->dx[12];
//   eskf->state.gyro_b[1] += eskf->dx[13];
//   eskf->state.gyro_b[2] += eskf->dx[14];

//   // clamp bias

//   eskf->state.gyro_b[0] = clamp(eskf->state.gyro_b[0], -0.05, 0.05);
//   eskf->state.gyro_b[1] = clamp(eskf->state.gyro_b[1], -0.05, 0.05);
//   eskf->state.gyro_b[2] = clamp(eskf->state.gyro_b[2], -0.05, 0.05);
//   eskf->state.acc_b[0] = clamp(eskf->state.acc_b[0], -0.2, 0.2);
//   eskf->state.acc_b[1] = clamp(eskf->state.acc_b[1], -0.2, 0.2);
//   eskf->state.acc_b[2] = clamp(eskf->state.acc_b[2], -0.2, 0.2);
// }

// static int eskf_update_cov() {

//   // covariance update
//   // symmetric form K(HPH.T+V)K.T
//   arm_mat_trans_f32(&K, &H); // m * 15
//   // status = arm_mat_add_f32(&HPH, &V, &HPH); // m * m
//   arm_mat_mult_f32(&K, &HPH, &K); // 15 * m
//   arm_mat_mult_f32(&K, &H, &KP);  // 15 * 15

//   // P <- P - K(HPH.T+V)K.T
//   arm_mat_sub_f32(&P, &KP, &P);

//   // Expanded Joseph

//   // arm_mat_mult_f32(&K, &H, &KP); // 15 * 15
//   // arm_mat_mult_f32(&KP, &P, &KP);

//   // arm_mat_sub_f32(&P, &KP, &P);

//   // arm_mat_trans_f32(&KP, &KP);
//   // arm_mat_sub_f32(&P, &KP, &P);

//   // Joseph form (I-KH)P(I-KH).T+KVK.T

//   // arm_mat_mult_f32(&K, &H, &KP); // 15 * 15

//   // memset(Q_data, 0, sizeof(Q_data));
//   // for (int i = 0; i < 15; i++) {
//   //   Q_data[i * Q.numCols + i] = 1;
//   // }

//   // arm_mat_sub_f32(&Q, &KP, &KP);
//   // arm_mat_trans_f32(&KP, &Ft);
//   // arm_mat_mult_f32(&KP, &P, &KP);
//   // arm_mat_mult_f32(&KP, &Ft, &KP);
//   // arm_mat_sub_f32(&P, &KP, &P);

//   // arm_mat_trans_f32(&K, &H); // m * 15
//   // arm_mat_mult_f32(&V, &H, &H);
//   // arm_mat_mult_f32(&K, &H, &KP);
//   // arm_mat_add_f32(&P, &KP, &P);

//   for (int i = 0; i < P.numCols; i++) {
//     if (!isfinite(P_data[i * P.numCols + i])) {
//       LOG_CRIT(0, "ESKF_COV_UPD_INF at %d", i * P.numCols + i < 0);
//       return -1;
//     }
//     if (P_data[i * P.numCols + i] < 0) {
//       LOG_CRIT(0, "ESKF_COV_UPD_SING at %d", i * P.numCols + i < 0);
//       return -1;
//     }
//   }

//   return 0;
// }

// static int eskf_update_cov_m3() {

//   // covariance update
//   // // symmetric form K(HPH.T+V)K.T
//   arm_mat_trans_f32(&K3, &H3);       // m * 15
//   arm_mat_mult_f32(&K3, &HPH3, &K3); // 15 * m
//   arm_mat_mult_f32(&K3, &H3, &KP);   // 15 * 15

//   // P <- P - K(HPH.T+V)K.T
//   arm_mat_sub_f32(&P, &KP, &P);

//   // Joseph form (I-KH)P(I-KH).T+KVK.T

//   // arm_mat_mult_f32(&K3, &H3, &KP); // 15 * 15

//   // memset(Q_data, 0, sizeof(Q_data));
//   // for (int i = 0; i < 15; i++) {
//   //   Q_data[i * Q.numCols + i] = 1;
//   // }

//   // arm_mat_sub_f32(&Q, &KP, &KP);
//   // arm_mat_trans_f32(&KP, &Ft);
//   // arm_mat_mult_f32(&KP, &P, &KP);
//   // arm_mat_mult_f32(&KP, &Ft, &KP);
//   // arm_mat_sub_f32(&P, &KP, &P);

//   // arm_mat_trans_f32(&K3, &H3); // m * 15
//   // arm_mat_mult_f32(&V3, &H3, &H3);
//   // arm_mat_mult_f32(&K3, &H3, &KP);
//   // arm_mat_add_f32(&P, &KP, &P);

//   for (int i = 0; i < P.numCols; i++) {
//     if (!isfinite(P_data[i * P.numCols + i])) {
//       LOG_CRIT(0, "ESKF_COV_UPD_INF at %d", i * P.numCols + i < 0);
//       return -1;
//     }
//     if (P_data[i * P.numCols + i] < 0) {
//       LOG_CRIT(0, "ESKF_COV_UPD_SING at %d", i * P.numCols + i < 0);
//       return -1;
//     }
//   }

//   return 0;
// }

// static int compute_kalman_gain() {

//   // K = P*Ht*(H*P*Ht+V)^-1
//   status = arm_mat_trans_f32(&H, &Ht);       // = 15*m
//   status = arm_mat_mult_f32(&H, &P, &HP);    // = m*15
//   status = arm_mat_mult_f32(&HP, &Ht, &HPH); // = m * m
//   status = arm_mat_add_f32(&HPH, &V, &HPH);  // m * m
//   memcpy(HPHic_data, HPH_data, sizeof(HPH_data));
//   status = arm_mat_inverse_f32(&HPHic, &HPHi); // = m * m

//   if (status == ARM_MATH_SINGULAR) {
//     LOG_ERR(0, "SINGULAR_MATRIX");
//     return -1;
//   }

//   status = arm_mat_mult_f32(&Ht, &HPHi, &Ht); // = 15 * m
//   status = arm_mat_mult_f32(&P, &Ht, &K);     // = 15 * m

//   return 0;
// }

// static int compute_kalman_gain_m3() {

//   // K = P*Ht*(H*P*Ht+V)^-1
//   status = arm_mat_trans_f32(&H3, &Ht3);        // = 15*m
//   status = arm_mat_mult_f32(&H3, &P, &HP3);     // = m*15
//   status = arm_mat_mult_f32(&HP3, &Ht3, &HPH3); // = m * m
//   status = arm_mat_add_f32(&HPH3, &V3, &HPH3);  // m * m
//   memcpy(HPHic3_data, HPH_data3, sizeof(HPH_data3));
//   status = arm_mat_inverse_f32(&HPHic3, &HPHi3); // = m * m

//   if (status == ARM_MATH_SINGULAR) {
//     LOG_ERR(0, "SINGULAR_MATRIX");
//     return -1;
//   }

//   status = arm_mat_mult_f32(&Ht3, &HPHi3, &Ht3); // = 15 * m
//   status = arm_mat_mult_f32(&P, &Ht3, &K3);      // = 15 * m

//   return 0;
// }

// int eskf_update_accel(eskf_t *eskf, const accel3d_t *acc_m) {
//   acc_body[0] = acc_m->accel_y;
//   acc_body[1] = acc_m->accel_x;
//   acc_body[2] = acc_m->accel_z;

//   float acc_abs = sqrtf(acc_body[0] * acc_body[0] + acc_body[1] * acc_body[1]
//   +
//                         acc_body[2] * acc_body[2]);
//   float vscale = 1.0f;

//   // if significant acceleration not reliable
//   if (acc_abs > 1.5) {
//     LOG_WARN(0, "ACC_UPD_SKIP");
//     return 0;
//   } else if (acc_abs > 1.2) {
//     vscale = 10.0f;
//   }

//   linv3(acc_body, acc_body, eskf->state.acc_b, 9.81f, -1);
//   // quat_rotate_vec(acc_glob, eskf->state.quat,
//   //                 acc_body); // body -> world

//   quat_inv(quat_inv_, eskf->state.quat);
//   quat_rotate_vec(g_body, quat_inv_, eskf->state.g);
//   linv3(acc_body, acc_body, g_body, 1, -1);
//   vec_skew(a_x, g_body);
//   mat_scale3x3(&Ra, 0, 0, -1);

//   memset(H_data3, 0, sizeof(H_data3));
//   set3x3(H_data3, 0, 6, a_x, ESKFN);
//   // H_data3[9] = 1;
//   // H_data3[15 + 10] = 1;
//   // H_data3[15 * 2 + 11] = 1;

//   // measurement error covariance
//   memset(V3_data, 0, sizeof(V3_data));
//   V3_data[0] = ESKF_SAN * vscale;
//   V3_data[m3 + 1] = ESKF_SAN * vscale;
//   V3_data[m3 + 2] = ESKF_SAN * vscale;

//   compute_kalman_gain_m3();

//   // compute error state change
//   arm_mat_vec_mult_f32(&K3, acc_body, eskf->dx);

//   eskf_update_cov_m3();

//   inject_error(eskf);

//   // reset error-state mean
//   memset(eskf->dx, 0, sizeof(eskf->dx));

//   return 0;
// }

// int eskf_update_yaw(eskf_t *eskf, mag3d_t *mag, float cov, float *yaw_m,
//                     float offv[3], float offM[3][3]) {
//   float mag_v[3] = {mag->MagY, mag->MagX, -mag->MagZ};
//   // Normalize magnetometer measurement: we only need orientation
//   // vec_norm(mag_meas);

//   float quat_rot[4];
//   float state_yaw, state_roll, state_pitch, yaw;

//   if (MAG_UPDATE_METHOD == MAG_UPDATE_YAW) {
//     quat_get_euler(eskf->state.quat, &state_roll, &state_pitch, &state_yaw);
//     euler_to_quat(quat_rot, state_roll, state_pitch, 0.0);

//     mag_v[0] -= offv[1];
//     mag_v[1] -= offv[0];
//     mag_v[2] -= -offv[2];

//     mag_v[0] =
//         offM[1][0] * mag_v[1] + offM[1][1] * mag_v[0] + offM[1][2] *
//         -mag_v[2];
//     mag_v[1] =
//         offM[0][0] * mag_v[1] + offM[0][1] * mag_v[0] + offM[0][2] *
//         -mag_v[2];
//     mag_v[2] =
//         offM[2][0] * mag_v[0] + offM[2][1] * mag_v[1] + offM[2][2] *
//         -mag_v[2];

//     quat_rotate_vec(mag_v, quat_rot, mag_v);
//     yaw = -atan2f(mag_v[1], mag_v[0]) + MAG_DECL;

//     if (yaw - state_yaw < -M_PI) {
//       eskf->dx[8] = yaw - state_yaw + 2 * M_PI;
//     } else if (yaw - state_yaw > M_PI) {
//       eskf->dx[8] = yaw - state_yaw - 2 * M_PI;
//     } else {
//       eskf->dx[8] = yaw - state_yaw; // atan2f(sinf(yaw - state_yaw),
//                                      // cosf(yaw - state_yaw));
//     }
//     *yaw_m = yaw;
//     memset(H_data, 0, sizeof(H_data));
//     H_data[8] = 1;
//   } else {
//   }
//   // measurement error covariance
//   memset(V_data, 0, sizeof(V_data));
//   V_data[0] = cov;

//   compute_kalman_gain();

//   // compute error state change
//   arm_mat_vec_mult_f32(&K, &eskf->dx[8], eskf->dx);

//   eskf_update_cov();

//   inject_error(eskf);

//   // reset error-state mean
//   memset(eskf->dx, 0, sizeof(eskf->dx));

//   return 0;
// }

// int eskf_update_gps(eskf_t *eskf, const GPS_data *data, int32_t home_lon,
//                     int32_t home_lat, float home_alt, float hacc, float vacc,
//                     float sacc, float pm[5]) {
//   float posvel_diff[5];
//   float pos_v[3];
//   if (home_lat != 0 && home_lon != 0) {
//     lla_to_ned(data->lat, data->lon, data->alt, home_lat, home_lon, home_alt,
//                pos_v);
//   } else {
//     return -1;
//   }

//   posvel_diff[0] = pos_v[0] - eskf->state.pos[0];
//   posvel_diff[1] = pos_v[1] - eskf->state.pos[1];
//   posvel_diff[2] = pos_v[2] + eskf->state.pos[2];
//   posvel_diff[3] = data->vn - eskf->state.vel[0];
//   posvel_diff[4] = data->ve - eskf->state.vel[1];

//   // compute kalman gain
//   memset(H_data3, 0, sizeof(H_data3));
//   H_data3[0] = 1;
//   H_data3[ESKFN * 1 + 1] = 1;
//   H_data3[ESKFN * 2 + 2] = -1;
//   H_data3[ESKFN * 3 + 3] = 1;
//   H_data3[ESKFN * 4 + 4] = 1;

//   // measurement error covariance
//   memset(V3_data, 0, sizeof(V3_data));
//   V3_data[m3 * 0 + 0] = ESKF_SGPS_POS;
//   V3_data[m3 * 1 + 1] = ESKF_SGPS_POS;
//   V3_data[m3 * 2 + 2] = ESKF_SGPS_POS;
//   V3_data[m3 * 3 + 3] = ESKF_SGPS_VEL;
//   V3_data[m3 * 4 + 4] = ESKF_SGPS_VEL;

//   compute_kalman_gain_m3();

//   // compute error state change
//   arm_mat_vec_mult_f32(&K3, posvel_diff, eskf->dx);

//   eskf_update_cov_m3();

//   // inject error-state
//   inject_error(eskf);

//   // reset error-state mean
//   memset(&eskf->dx, 0, sizeof(eskf->dx));

//   return 0;
// }

// int eskf_update_baro(eskf_t *eskf, float alt, float cov) {
//   float alt_v[1] = {-alt - eskf->state.pos[2]};

//   memset(H_data, 0, sizeof(H_data));
//   H_data[2] = 1; // updates z component of position

//   // measurement error covariance
//   memset(V_data, 0, sizeof(V_data));
//   V_data[0] = cov;

//   compute_kalman_gain();

//   // compute error state change
//   arm_mat_vec_mult_f32(&K, alt_v, eskf->dx);

//   eskf_update_cov();

//   inject_error(eskf);

//   // reset error-state mean
//   memset(&eskf->dx, 0, sizeof(eskf->dx));

//   return 0;
// }

// int eskf_update_vbaro(eskf_t *eskf, float alt, float cov, float dt) {
//   static float alt_prev = 0;
//   float alt_v[1] = {-(alt - alt_prev) / dt - eskf->state.vel[2]};
//   alt_prev = alt;

//   memset(H_data, 0, sizeof(H_data));
//   H_data[5] = 1; // updates z component of velocity

//   // measurement error covariance
//   memset(V_data, 0, sizeof(V_data));
//   V_data[0] = cov;

//   compute_kalman_gain();

//   // compute error state change
//   arm_mat_vec_mult_f32(&K, alt_v, eskf->dx);

//   eskf_update_cov();

//   inject_error(eskf);

//   // reset error-state mean
//   memset(&eskf->dx, 0, sizeof(eskf->dx));

//   return 0;
// }

// void eskf_update_dist_sensor(eskf_t *eskf, float alt, float cov) {}

// void eskf_reset(eskf_t *eskf) { memset(&eskf->dx, 0, sizeof(eskf->dx)); }

// void eskf_inject(eskf_t *eskf, const eskf_state_t *es) {
//   linv3(eskf->state.pos, eskf->state.pos, es->pos, 1, 1);
//   linv3(eskf->state.vel, eskf->state.vel, es->vel, 1, 1);
//   quat_mul(eskf->state.quat, eskf->state.quat, es->quat);
//   linv3(eskf->state.acc_b, eskf->state.acc_b, es->acc_b, 1, 1);
//   linv3(eskf->state.gyro_b, eskf->state.gyro_b, es->gyro_b, 1, 1);
// }

// void eskf_get_cov_orientation(eskf_t *eskf, float dst[9]) {
//   dst[0] = P_data[ESKFN * 6 + 6];
//   dst[1] = P_data[ESKFN * 6 + 7];
//   dst[2] = P_data[ESKFN * 6 + 8];
//   dst[3] = P_data[ESKFN * 7 + 6];
//   dst[4] = P_data[ESKFN * 7 + 7];
//   dst[5] = P_data[ESKFN * 7 + 8];
//   dst[6] = P_data[ESKFN * 8 + 6];
//   dst[7] = P_data[ESKFN * 8 + 7];
//   dst[8] = P_data[ESKFN * 8 + 8];
// }

// void eskf_get_cov_bias(eskf_t *eskf, float dst[6]) {
//   dst[0] = P_data[ESKFN * 9 + 9];
//   dst[1] = P_data[ESKFN * 10 + 10];
//   dst[2] = P_data[ESKFN * 11 + 11];
//   dst[3] = P_data[ESKFN * 12 + 12];
//   dst[4] = P_data[ESKFN * 13 + 13];
//   dst[5] = P_data[ESKFN * 14 + 14];
// }

// void eskf_get_cov_posvel(eskf_t *eskf, float dst[21]) {
//   size_t idx = 0;
//   for (size_t j = 0; j < 6; j++) {
//     for (size_t i = j; i < 6; i++) {
//       dst[idx++] = P_data[j * ESKFN + i];
//     }
//   }
// }

// void eskf_get_cov_pos(eskf_t *eskf, float dst[9]) {
//   for (size_t i = 0; i < 9; i++) {
//     dst[i] = P_data[(i / 3) * ESKFN + i % 3];
//   }
// }
