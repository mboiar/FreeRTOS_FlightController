#include "Tasks.h"
#include "dshot.h"
#include "motor.h"
#include "pid.h"
#include "quat_utils.h"
#include "tim.h"

#define DSHOT_TYPE DSHOT300

#define CTRL_TYPE_PWM 0
#define CTRL_TYPE_DSHOT 1
#define CTRL_TYPE CTRL_TYPE_PWM

#define CRSF_TO_DSHOT(x)                                                       \
  (((x - RC_VAL_MIN) * DSHOT_RANGE) / (RC_VAL_MAX - RC_VAL_MIN) + DSHOT_VAL_MID)

static pid3d_s pid_rate, pid_vel, pid_att;
// pid_pos;
static vec3df rate_out;
static setpoint_t sp_rate;
// sp_pos, rate_out, pos_out;
static vec3df att_sp, state_att;
// static vec3df rc_rate;
static motors_pwm_s motors_pwm;

// static TickType_t last_time_ticks, now_ticks;
static float dt, inner_dt;

static uint16_t dshot_data;
// static sensor_data_t cur_imu_data;

static float acc_cmd[3], vel_cmd_ned[3], pid_out[3];

static uint32_t notif;

static uint32_t arm_cnt;

static float quat[4];

static mavlink_message_t msg;

// rc_scaled_t rc_scaled;

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

void rate_pid_loop() {
  rate_out.roll = pid_compute(&pid_rate.pid_x, gyrof[0] - eskf.state.gyro_b[0],
                              sp_rate.roll, dt);
  rate_out.pitch = pid_compute(&pid_rate.pid_y, gyrof[1] - eskf.state.gyro_b[1],
                               sp_rate.pitch, dt);
  rate_out.yaw =
      1.0f * pid_compute(&pid_rate.pid_z, gyrof[2] - eskf.state.gyro_b[2],
                         sp_rate.yaw, dt);
}

void vel_pid_loop(float vel_cmd[3], float dt) {
  acc_cmd[0] =
      pid_compute(&pid_vel.pid_x, eskf.state.vel[0], vel_cmd[0], dt); // dt?
  acc_cmd[1] = pid_compute(&pid_vel.pid_y, eskf.state.vel[1], vel_cmd[1], dt);
  acc_cmd[2] = pid_compute(&pid_vel.pid_z, eskf.state.vel[2], vel_cmd[2], dt);
  clamp(acc_cmd[0], -2, 2);
  clamp(acc_cmd[1], -2, 2);
  clamp(acc_cmd[2], -1, 1);
  sp_rate.throttle = sqrtf(acc_cmd[0] * acc_cmd[0] + acc_cmd[1] * acc_cmd[1] +
                           acc_cmd[2] * acc_cmd[2]) +
                     0.4f;

  att_sp.roll = atan2f(acc_cmd[1], acc_cmd[2]);
  att_sp.pitch = atan2f(
      -acc_cmd[0], sqrtf(acc_cmd[2] * acc_cmd[2] + acc_cmd[1] * acc_cmd[1]));
}

void att_pid_loop(float dt) {
  quat_get_euler(eskf.state.quat, &state_att.roll, &state_att.pitch,
                 &state_att.yaw);
  sp_rate.roll = pid_compute(&pid_att.pid_x, state_att.roll, att_sp.roll,
                             dt); // dt?
  sp_rate.pitch =
      pid_compute(&pid_att.pid_y, state_att.pitch, att_sp.pitch, dt);
  sp_rate.yaw = 1.0f * pid_compute(&pid_att.pid_z, state_att.yaw,
                                   state_att.yaw + att_sp.yaw, dt);
}

void pid_log(uint32_t tick) {

  // mavlink_msg_manual_setpoint_pack(
  //     fc_state.system_id, fc_state.comp_id, &msg, tick, sp_rate.roll,
  //     sp_rate.pitch, sp_rate.yaw, sp_rate.throttle, (uint8_t)rc_scaled.mode,
  //     0);
  // comm_tx_send(&msg);

  pid_out[0] = rate_out.roll;
  pid_out[1] = rate_out.pitch;
  pid_out[2] = rate_out.yaw;

  // mavlink_msg_debug_float_array_pack(fc_state.system_id, fc_state.comp_id,
  // &msg,
  //                                    tick, "PID_OUT", 0, pid_out);
  // mavlink_msg_debug_vect_pack(fc_state.system_id, fc_state.comp_id, &msg,
  //                             "PID_OUT", tick, rate_out.roll, rate_out.pitch,
  //                             rate_out.yaw);
  // comm_tx_send(&msg);
  // mavlink_msg_servo_output_raw_pack(fc_state.system_id, fc_state.comp_id,
  // &msg,
  //                                   tick, 0, motors_pwm.bl, motors_pwm.br,
  //                                   motors_pwm.fl, motors_pwm.fr, 0, 0, 0, 0,
  //                                   0, 0, 0, 0, 0, 0, 0, 0);
  // comm_tx_send(&msg);
  mavlink_msg_highres_imu_pack(
      1, MAV_COMP_ID_AUTOPILOT1, &msg, tick, rate_out.roll, rate_out.pitch,
      rate_out.yaw, euler[0], euler[1], euler[2],
      gyrof[0] - eskf.state.gyro_b[0], gyrof[1] - eskf.state.gyro_b[1],
      gyrof[2] - eskf.state.gyro_b[2], acc_f[0], acc_f[1], acc_f[2], 0, 0xFFFF,
      0);

  comm_tx_send(&msg);
}

void set_pwm_out() {
  motors_pwm.fr = PWM_MIN + clamp(sp_rate.throttle + rate_out.yaw -
                                      rate_out.roll + rate_out.pitch,
                                  0, 1) *
                                (PWM_MAX - PWM_MIN);
  motors_pwm.fl = PWM_MIN + clamp(sp_rate.throttle - rate_out.yaw +
                                      rate_out.roll + rate_out.pitch,
                                  0, 1) *
                                (PWM_MAX - PWM_MIN);
  motors_pwm.br = PWM_MIN + clamp(sp_rate.throttle - rate_out.yaw -
                                      rate_out.roll - rate_out.pitch,
                                  0, 1) *
                                (PWM_MAX - PWM_MIN);
  motors_pwm.bl = PWM_MIN + clamp(sp_rate.throttle + rate_out.yaw +
                                      rate_out.roll - rate_out.pitch,
                                  0, 1) *
                                (PWM_MAX - PWM_MIN);
}

void write_pwm_vals() {
  if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
    // dshot_data = CRSF_TO_DSHOT(rc_data.ch_data[0]);
    dshot_data = 70;
    int res = dshot_write(dshot_data, 1, TIM_CHANNEL_1);
    res = dshot_write(dshot_data, 1, TIM_CHANNEL_2);
    res = dshot_write(dshot_data, 1, TIM_CHANNEL_3);
    res = dshot_write(dshot_data, 1, TIM_CHANNEL_4);
  } else {
    pwm_set_pulse_us(&htim1, TIM_CHANNEL_1,
                     (uint32_t)clamp(motors_pwm.bl, 1000, 2000));
    pwm_set_pulse_us(&htim1, TIM_CHANNEL_2,
                     (uint32_t)clamp(motors_pwm.br, 1000, 2000));
    pwm_set_pulse_us(&htim1, TIM_CHANNEL_3,
                     (uint32_t)clamp(motors_pwm.fl, 1000, 2000));
    pwm_set_pulse_us(&htim1, TIM_CHANNEL_4,
                     (uint32_t)clamp(motors_pwm.fr, 1000, 2000));
  }
}

void init_motors() {

  if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
    dshot_init(DSHOT300);
    for (int i = 0; i < 1000; i++) {
      dshot_write(0, 0, TIM_CHANNEL_2);
      // vTaskDelay(pdMS_TO_TICKS(1));
    }
  } else {
    pwm_init();
    // arming
    motors_pwm.bl = PWM_MIN;
    motors_pwm.br = PWM_MIN;
    motors_pwm.fl = PWM_MIN;
    motors_pwm.fr = PWM_MIN;
    pwm_set_all(&motors_pwm);
    vTaskDelay(pdMS_TO_TICKS(2));
    motors_pwm.bl = PWM_MAX;
    motors_pwm.br = PWM_MAX;
    motors_pwm.fl = PWM_MAX;
    motors_pwm.fr = PWM_MAX;
    pwm_set_all(&motors_pwm);
    vTaskDelay(pdMS_TO_TICKS(2));
    motors_pwm.bl = PWM_MIN;
    motors_pwm.br = PWM_MIN;
    motors_pwm.fl = PWM_MIN;
    motors_pwm.fr = PWM_MIN;
    pwm_set_all(&motors_pwm);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void deinit_motors() {
  if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
    // TODO
  } else {
    pwm_deinit();
  }
}

void arm() {
  LOG_INFO(0, "ARMING");
  if (!(fc_state.mode & MAV_MODE_FLAG_HIL_ENABLED)) {
    init_motors();
  }
  fc_state.state = MAV_STATE_ACTIVE;
}

void disarm() {
  LOG_INFO(0, "DISARMING");
  if (!(fc_state.mode & MAV_MODE_FLAG_HIL_ENABLED)) {
    deinit_motors();
  }
  fc_state.state = MAV_STATE_STANDBY;
}

/**
 * @brief Flight Loop
 * @param argument: Not used
 * @retval None
 */
void TaskFlightLoop(void *argument) {

  // TODO: tune PID parameters
  pid_init(&pid_rate.pid_x, 0.005f, 0.0000f, 0.000f, -0.3f, 0.3f);
  pid_init(&pid_rate.pid_y, 0.005f, 0.0000f, 0.000f, -0.3f, 0.3f);
  pid_init(&pid_rate.pid_z, 0.005f, 0.0000f, 0.000f, -0.3f, 0.3f);
  pid_init(&pid_vel.pid_x, 0.5f, 0.0f, 0.0f, -0.2f, 0.2f);
  pid_init(&pid_vel.pid_y, 0.5f, 0.0f, 0.0f, -0.2f, 0.2f);
  pid_init(&pid_vel.pid_z, 1.0f, 0.0f, 0.0f, -0.1f, 0.1f);
  pid_init(&pid_att.pid_x, 20.0f, 0.0f, 0.0f, -10.0f, 10.0f);
  pid_init(&pid_att.pid_y, 20.0f, 0.0f, 0.0f, -10.0f, 10.0f);
  pid_init(&pid_att.pid_z, 20.0f, 0.0f, 0.0f, -10.0f, 10.0f);

  static uint32_t last_tick, tick, last_inner_tick, loop_cnt;

  last_tick = get_time_since_boot_us();
  last_inner_tick = get_time_since_boot_us();

  arm_cnt = 0;
  loop_cnt = 0;

  for (;;) {
    // run with 200 Hz freq
    if (xTaskNotifyWait(pdFALSE, ULONG_MAX, &notif, portMAX_DELAY) == pdTRUE) {
      if (notif & PID_COMPUTE) {

        notif &= ~PID_COMPUTE;

        if (fc_state.state == MAV_STATE_FLIGHT_TERMINATION) {
          disarm();
        }

        if (!((fc_state.state & MAV_STATE_ACTIVE) ||
              fc_state.state & MAV_STATE_STANDBY)) {
          continue;
        }
        tick = get_time_since_boot_us();
        dt = (float)(tick - last_tick) / 1000000.0f;
        last_tick = tick;
        loop_cnt++;

        // get latest sensor data
        // if (imu_mutex != NULL) {
        //   xSemaphoreTake(imu_mutex, portMAX_DELAY);
        //   cur_imu_data = imu_data;
        //   xSemaphoreGive(imu_mutex);
        // }

        // inner PID

        if (arm_cnt == 400) {
          if (fc_state.state == MAV_STATE_STANDBY) {
            fc_state.state = MAV_STATE_CALIBRATING;
          } else if (fc_state.state == MAV_STATE_ACTIVE) {
            disarm();
          }
          arm_cnt = 0;
        }

        switch (fc_state.custom_mode) {
        default:
        case FLIGHT_MODE_ACRO:

          if (rc_scaled.arm && rc_scaled.throttle == 0) {
            arm_cnt++;
            continue;
          } else {
            arm_cnt = 0;
          }

          sp_rate.roll = rc_scaled.roll / 180.0f * (float)M_PI;
          sp_rate.pitch = rc_scaled.pitch / 180.0f * (float)M_PI;
          sp_rate.yaw = rc_scaled.yaw / 180.0f * (float)M_PI;
          sp_rate.throttle = rc_scaled.throttle;
          sp_rate.ts = rc_scaled.ts;

          break;

        case FLIGHT_MODE_STABILIZED:

          if (rc_scaled.arm && rc_scaled.throttle == 0) {
            arm_cnt++;
            continue;
          } else {
            arm_cnt = 0;
          }

          att_sp.roll = rc_scaled.roll / 180.0f * (float)M_PI;
          att_sp.pitch = rc_scaled.pitch / 180.0f * (float)M_PI;
          att_sp.yaw = rc_scaled.yaw / 180.0f * (float)M_PI;
          sp_rate.throttle = rc_scaled.throttle;
          sp_rate.ts = rc_scaled.ts;

          // 50 Hz
          if (loop_cnt % 5 == 0) {
            // attitude pid
            inner_dt = (float)(tick - last_inner_tick) / 1000000.0f;
            last_inner_tick = tick;

            att_pid_loop(inner_dt);
          }
          break;

        case FLIGHT_MODE_GUIDED:
          // desired velocity -> desired acceleration NED
          if (notif & PID_SET_TARGET_VELOCITY) {
            notif &= ~PID_SET_TARGET_VELOCITY;

            inner_dt = (float)(tick - last_inner_tick) / 1000000.0f;
            last_inner_tick = tick;
            // quat_rotate_vec(vel_body, eskf.state.quat,
            //                 eskf.state.vel); // world -> body
            sp_rate.yaw = vel_cmd[3];
            sp_rate.ts = tick; // todo: timestamp
            euler_to_quat(quat, state_att.pitch, state_att.roll, 0);
            quat_rotate_vec(vel_cmd_ned, quat, vel_cmd);
            vel_pid_loop(vel_cmd_ned, inner_dt);

          } else {
            if (tick - last_inner_tick > 1000000) {
              memset(vel_cmd_ned, 0, sizeof(vel_cmd_ned));
            }
          }

          // 50 Hz
          if (loop_cnt % 4 == 0) {
            // attitude pid
            att_pid_loop(dt * 4);
          }

          break;

        case FLIGHT_MODE_POSHOLD:

          if (rc_scaled.arm && rc_scaled.throttle == 0) {
            arm_cnt++;
            continue;
          } else {
            arm_cnt = 0;
          }

          sp_rate.yaw = rc_scaled.yaw / 180.0f * (float)M_PI;
          sp_rate.ts = rc_scaled.ts;

          // 10 Hz
          if (loop_cnt % 20 == 0) {
            vel_cmd_ned[0] = rc_scaled.roll;
            vel_cmd_ned[1] = rc_scaled.pitch;
            vel_cmd_ned[2] = rc_scaled.throttle;

            vel_pid_loop(vel_cmd_ned, dt * 20);
          }

          if (loop_cnt % 4 == 0) {
            // attitude pid

            att_pid_loop(dt * 4);
          }
          break;

        case FLIGHT_MODE_LAND_UNSUPPORTED:
          sp_rate.yaw = rc_scaled.yaw / 180.0f * (float)M_PI;
          sp_rate.ts = rc_scaled.ts;

          if (arm_cnt == 100) {
            arm_cnt = 0;
            disarm();
          }

          // check if landed
          if (eskf.state.vel[2] < 0.3f) {
            arm_cnt++;
            sp_rate.throttle = 0;
            continue;
          } else if (fc_state.sensors_enabled) {
            // TODO: check GPS
            // 10 Hz
            if (loop_cnt % 20 == 0) {
              vel_cmd_ned[0] = 0;
              vel_cmd_ned[1] = 0;
              vel_cmd_ned[2] = -0.5f;

              vel_pid_loop(vel_cmd_ned, dt * 20);
            }
          } else {
            sp_rate.throttle = 0.4f; // slightly below hover throttle
          }
          arm_cnt = 0;

          if (loop_cnt % 4 == 0) {
            // attitude pid
            att_sp.roll = 0;
            att_sp.pitch = 0;
            att_pid_loop(dt * 4);
          }
          break;
        }

        // reject old setpoints
        // if (tick - sp_rate.ts < 50000) {
        rate_pid_loop();
        set_pwm_out();

        if (!(fc_state.mode & MAV_MODE_FLAG_HIL_ENABLED) &&
            (fc_state.state == MAV_STATE_ACTIVE)) {
          write_pwm_vals();
        }
        // }
        if (loop_cnt % 4 == 0) {

          if (PID_DEBUG) {
            pid_log(tick);

            // if (tick - rc_scaled.ts > 50000) {
            //   LOG_ERR(0, "EXPIRED_RC %lu %lu", tick, rc_scaled.ts);
            //   // continue;
            // }
          }
        }
      }

      if (notif & PID_ARM_READY) {
        notif &= ~PID_ARM_READY;

        // TODO: get battery state from sensor not esc
        if (fc_state.battery_state == MAV_BATTERY_CHARGE_STATE_OK ||
            fc_state.battery_state == MAV_BATTERY_CHARGE_STATE_UNDEFINED) {
          arm();
        }
      }
    }
  }
}