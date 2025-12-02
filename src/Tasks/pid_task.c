#include "Tasks.h"
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
  if (CTRL_TYPE == CTRL_TYPE_DSHOT) {
    dshot_init(DSHOT300);
    for (int i = 0; i < 1000; i++) {
      dshot_write(0, 0, TIM_CHANNEL_2);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  } else {
    init_pwm();
    // arming
    pwm_set_pulse_us(&htim1, TIM_CHANNEL_2, 1000); // idle
  }

  uint32_t tick = 0;

  // dshot_set_direction(0, TIM_CHANNEL_1);

  for (;;) {
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
        pwm_set_pulse_us(&htim1, TIM_CHANNEL_2,
                         1000); // small throttle test
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