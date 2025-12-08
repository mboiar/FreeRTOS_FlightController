#include "Tasks.h"
#include "motor.h"
#include "pid.h"

static pid3d_s pid_rate, pid_pos;
static vec3df sp_rate, sp_pos, rate_out, pos_out;
static vec3df rate, pos;
static vec3df rc_rate;
static motors_pwm_s motors_pwm;
static uint16_t sp_throttle;

TickType_t last_time_ticks, now_ticks;
static float dt;
#include "dshot.h"
#include "tim.h"

#define DSHOT_TYPE DSHOT300

static uint16_t dshot_data;
static sensor_data_t cur_imu_data;

// rc_scaled_t rc_scaled;

#define CRSF_TO_DSHOT(x)                                                       \
  (((x - RC_VAL_MIN) * DSHOT_RANGE) / (RC_VAL_MAX - RC_VAL_MIN) + DSHOT_VAL_MID)

void init_pwm() {
  htim1.Instance->PSC = 99;
  htim1.Instance->ARR = 19999;
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void pwm_set_pulse_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t us) {
  if (us < 500)
    us = 500;
  if (us > 2500)
    us = 2500;
  __HAL_TIM_SET_COMPARE(htim, channel, us);
}

#define CTRL_TYPE_PWM 0
#define CTRL_TYPE_DSHOT 1
#define CTRL_TYPE CTRL_TYPE_DSHOT

/**
 * @brief Flight Loop
 * @param argument: Not used
 * @retval None
 */
void TaskFlightLoop(void *argument) {

  // TODO: tune PID parameters
  pid_init(&pid_rate.pid_yaw, 1, 0.01, 1, -100, 100);
  pid_init(&pid_rate.pid_roll, 1, 0.01, 1, -100, 100);
  pid_init(&pid_rate.pid_pitch, 1, 0.01, 1, -100, 100);
  pid_init(&pid_pos.pid_pitch, 1, 0.01, 1, -100, 100);
  pid_init(&pid_pos.pid_roll, 1, 0.01, 1, -100, 100);

  if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
    dshot_init(DSHOT300);
    for (int i = 0; i < 1000; i++) {
      dshot_write(0, 0, TIM_CHANNEL_2);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  } else {
    // init_pwm();
    // arming
    // pwm_set_pulse_us(&htim1, TIM_CHANNEL_2, 1000); // idle
  }

  uint32_t tick = 0;

  // dshot_set_direction(0, TIM_CHANNEL_1);

  for (;;) {

    now_ticks = xTaskGetTickCount();
    dt = portTICK_PERIOD_MS * (now_ticks - last_time_ticks) / 1000.0f;
    last_time_ticks = now_ticks;

    // TODO: Update SP and measurements

    sp_rate.yaw = pid_compute(&pid_rate.pid_yaw, rate.yaw, sp_pos.yaw, dt);
    sp_rate.roll = pid_compute(&pid_pos.pid_roll, pos.roll, sp_pos.roll, dt);
    sp_rate.pitch =
        pid_compute(&pid_pos.pid_pitch, pos.pitch, sp_pos.pitch, dt);
    rate_out.roll =
        pid_compute(&pid_rate.pid_roll, rate.roll, sp_rate.roll, dt);
    rate_out.pitch =
        pid_compute(&pid_rate.pid_pitch, rate.pitch, sp_rate.pitch, dt);

    motors_pwm.fr = sp_throttle - sp_rate.yaw + rate_out.roll + rate_out.pitch;
    motors_pwm.fl = sp_throttle + sp_rate.yaw - rate_out.roll + rate_out.pitch;
    motors_pwm.br = sp_throttle + sp_rate.yaw + rate_out.roll - rate_out.pitch;
    motors_pwm.bl = sp_throttle - sp_rate.yaw - rate_out.roll - rate_out.pitch;

    // TODO: Set ESC output
    // setVehiclePWM(&motors_pwm);

    vTaskDelay(pdMS_TO_TICKS(5));
    // run with 1 kHz freq
    if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) == pdTRUE) {
      tick++;

      // get latest sensor data
      // if (imu_mutex != NULL) {
      //   xSemaphoreTake(imu_mutex, portMAX_DELAY);
      //   cur_imu_data = imu_data;
      //   xSemaphoreGive(imu_mutex);
      // }

      // inner PID TODO

      if (tick % 5 == 0) {
        // outer PID TODO
      }
      if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
        // dshot_data = CRSF_TO_DSHOT(rc_data.ch_data[0]);
        dshot_data = 69;
        int res = dshot_write(dshot_data, 0, TIM_CHANNEL_2);
      } else {
        // pwm_set_pulse_us(&htim1, TIM_CHANNEL_2,
        //                  1000); // small throttle test
      }

      // dshot_data = 1000 + (dshot_data - DSHOT_VAL_MIN) / DSHOT_RANGE * 1000;
      // if (dshot_data < 2000 && dshot_data > 1000) {
      //   pwm_set_pulse_us(&htim1, TIM_CHANNEL_2,
      //                    dshot_data); // small throttle test
      // } else {

      // }
      // if (tick == 4000) {
      //   pwm_set_pulse_us(&htim1, TIM_CHANNEL_2, 1000); // small throttle test
      // }

      // res = dshot_write(dshot_data, 0, TIM_CHANNEL_2);
      // res = dshot_write(dshot_data, 0, TIM_CHANNEL_3);
      // res = dshot_write(dshot_data, 0, TIM_CHANNEL_4);
    }
  }
}