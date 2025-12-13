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

static pid3d_s pid_rate, pid_pos;
// pid_pos;
static vec3df sp_rate, rate_out;
// sp_pos, rate_out, pos_out;
static vec3df rate, att;
static vec3df rc_rate;
static motors_pwm_s motors_pwm;
static float sp_throttle;

TickType_t last_time_ticks, now_ticks;
static float dt, inner_dt;

static uint16_t dshot_data;
static sensor_data_t cur_imu_data;

static float vel_body[3];

static uint32_t notif;

// rc_scaled_t rc_scaled;

void pwm_set_pulse_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t us) {
  if (us < PWM_MIN)
    us = PWM_MIN;
  if (us > PWM_MAX)
    us = PWM_MAX;
  __HAL_TIM_SET_COMPARE(htim, channel, us);
}

/**
 * @brief Flight Loop
 * @param argument: Not used
 * @retval None
 */
void TaskFlightLoop(void *argument) {
  mavlink_message_t msg;

  // TODO: tune PID parameters
  pid_init(&pid_rate.pid_x, 0.01, 0.0, 0, -100, 100);
  pid_init(&pid_rate.pid_y, 0.01, 0.0, 0, -100, 100);
  pid_init(&pid_rate.pid_z, 0.01, 0.0, 0, -100, 100);
  pid_init(&pid_pos.pid_x, 0.01, 0.0, 0.0, -100, 100);
  pid_init(&pid_pos.pid_y, 0.01, 0.0, 0.0, -100, 100);
  pid_init(&pid_pos.pid_z, 0.01, 0.0, 0.0, -100, 100);

  if (!(fc_state.mode & MAV_MODE_FLAG_HIL_ENABLED)) {
    if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
      dshot_init(DSHOT300);
      for (int i = 0; i < 1000; i++) {
        dshot_write(0, 0, TIM_CHANNEL_2);
        vTaskDelay(pdMS_TO_TICKS(1));
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

  uint32_t last_tick = get_time_since_boot_us(), tick, last_inner_tick;

  // dshot_set_direction(0, TIM_CHANNEL_1);

  for (;;) {

    // run with 200 Hz freq
    if (xTaskNotifyWait(pdFALSE, 0, &notif, portMAX_DELAY) == pdTRUE) {
      if (notif & PID_COMPUTE) {
        notif &= ~PID_COMPUTE;

        if (!(fc_state.state & MAV_STATE_ACTIVE)) {
          continue;
        }
        tick = get_time_since_boot_us();
        dt = (float)(tick - last_tick) / 1000000.0f;
        last_tick = tick;

        // get latest sensor data
        // if (imu_mutex != NULL) {
        //   xSemaphoreTake(imu_mutex, portMAX_DELAY);
        //   cur_imu_data = imu_data;
        //   xSemaphoreGive(imu_mutex);
        // }

        // inner PID

        switch (fc_state.custom_mode) {
        case FLIGHT_MODE_ACRO:
          // check if expired
          if (tick - rc_scaled.ts > 50000) {
            // TODO log
            // continue;
          }
          sp_rate.roll = rc_scaled.roll / 180.0f * M_PI;
          sp_rate.pitch = rc_scaled.pitch / 180.0f * M_PI;
          sp_rate.yaw = rc_scaled.yaw / 180.0f * M_PI;
          sp_throttle = rc_scaled.throttle;
          rate_out.roll = pid_compute(
              &pid_rate.pid_x, imu_data.gyro.gyro_y - eskf.state.gyro_b[0],
              sp_rate.roll, dt);
          rate_out.pitch = pid_compute(
              &pid_rate.pid_y, imu_data.gyro.gyro_x - eskf.state.gyro_b[1],
              sp_rate.pitch, dt);
          rate_out.yaw = pid_compute(
              &pid_rate.pid_z, imu_data.gyro.gyro_z - eskf.state.gyro_b[2],
              sp_rate.yaw, dt);
          mavlink_msg_manual_setpoint_pack(
              fc_state.system_id, fc_state.comp_id, &msg, tick, sp_rate.roll,
              sp_rate.pitch, sp_rate.yaw, sp_throttle, rc_scaled.mode, 0);
          comm_tx_send(&msg);
          float pid_out[3] = {rate_out.roll, rate_out.pitch, rate_out.yaw};
          mavlink_msg_debug_float_array_pack(fc_state.system_id,
                                             fc_state.comp_id, &msg, tick,
                                             "PID output", 0, pid_out);
          comm_tx_send(&msg);

          break;
        case FLIGHT_MODE_ALTHOLD:
          /* code */
          // continue;
          break;
        case FLIGHT_MODE_GUIDED:

          break;
        default:
          // continue;
          break;
        }

        if (notif & PID_SET_TARGET_VELOCITY &&
            fc_state.custom_mode == FLIGHT_MODE_GUIDED) {
          quat_rotate_vec(vel_body, eskf.state.quat,
                          vel_body); // body->world
          att.roll =
              pid_compute(&pid_pos.pid_x, vel_body[0], vel_cmd[0], dt); // dt?
          att.pitch = pid_compute(&pid_pos.pid_y, vel_body[1], vel_cmd[1], dt);
          att.yaw = pid_compute(&pid_pos.pid_z, vel_body[2], vel_cmd[2], dt);
        }

        // 50 Hz
        if (tick % 4 == 0) {
          // outer PID
          // velocity PID
          inner_dt = (float)(tick - last_inner_tick) / 1000000.0f;
          last_inner_tick = tick;
          rate.roll =
              pid_compute(&pid_pos.pid_x, vel_body[0], vel_cmd[0], dt); // dt?
          rate.pitch = pid_compute(&pid_pos.pid_y, vel_body[1], vel_cmd[1], dt);
          rate.yaw = pid_compute(&pid_pos.pid_z, vel_body[2], vel_cmd[2], dt);
        }

        motors_pwm.fr = PWM_MIN + clamp(sp_throttle + sp_rate.yaw +
                                            rate_out.roll + rate_out.pitch,
                                        0, 1) *
                                      (PWM_MAX - PWM_MIN);
        motors_pwm.fl = PWM_MIN + clamp(sp_throttle - sp_rate.yaw -
                                            rate_out.roll + rate_out.pitch,
                                        0, 1) *
                                      (PWM_MAX - PWM_MIN);
        motors_pwm.br = PWM_MIN + clamp(sp_throttle - sp_rate.yaw +
                                            rate_out.roll - rate_out.pitch,
                                        0, 1) *
                                      (PWM_MAX - PWM_MIN);
        motors_pwm.bl = PWM_MIN + clamp(sp_throttle + sp_rate.yaw -
                                            rate_out.roll - rate_out.pitch,
                                        0, 1) *
                                      (PWM_MAX - PWM_MIN);

        mavlink_msg_servo_output_raw_pack(
            fc_state.system_id, fc_state.comp_id, &msg, tick, 0, motors_pwm.bl,
            motors_pwm.br, motors_pwm.fl, motors_pwm.fr, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0);
        comm_tx_send(&msg);

        if (!(fc_state.mode & MAV_MODE_FLAG_HIL_ENABLED)) {
          if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
            // dshot_data = CRSF_TO_DSHOT(rc_data.ch_data[0]);
            dshot_data = 69;
            int res = dshot_write(dshot_data, 0, TIM_CHANNEL_1);
            res = dshot_write(dshot_data, 0, TIM_CHANNEL_2);
            res = dshot_write(dshot_data, 0, TIM_CHANNEL_3);
            res = dshot_write(dshot_data, 0, TIM_CHANNEL_4);
          } else {
            pwm_set_pulse_us(&htim1, TIM_CHANNEL_1,
                             clamp(motors_pwm.bl, 1000, 2000));
            pwm_set_pulse_us(&htim1, TIM_CHANNEL_2,
                             clamp(motors_pwm.br, 1000, 2000));
            pwm_set_pulse_us(&htim1, TIM_CHANNEL_3,
                             clamp(motors_pwm.fl, 1000, 2000));
            pwm_set_pulse_us(&htim1, TIM_CHANNEL_4,
                             clamp(motors_pwm.fr, 1000, 2000));
          }
        }
      }
    }
  }
}