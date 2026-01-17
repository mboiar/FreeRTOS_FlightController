
#include "hcsr04.h"
#include "FreeRTOS.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tasks.h"
#include "tim.h"
#include <stdbool.h>
#include <stdint.h>

void hcsr04_init(hcsr04_sensor_t *sensor) {
  // dwt_init();

  sensor->state = 0;
  sensor->t_start_us = 0;
  sensor->duration_us = 0;
  sensor->last_trigger_us = 0;
  sensor->last_distance_cm = -1.0f;

  /* Ensure TRIG pin low */
  HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}

void hcsr04_reset(hcsr04_sensor_t *sensor) {
  sensor->state = 1; /* WAIT_RISING */
  sensor->t_start_us = 0;
  sensor->duration_us = 0;
  sensor->last_trigger_us = htim3.Instance->CNT;
}

void hcsr04_trigger() {
  HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
  TIM3->CCR2 = TIM3->CNT + 100;
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);
}

float duration_to_dist(float dur_us, float temperature) {
  float soundSpeed = (331.4f + (0.606f * temperature));
  return soundSpeed * dur_us / 100000.0f / 2.0f;
}

static float median5(const float *arr) {
  float t, a = arr[0], b = arr[1], c = arr[2], d = arr[3], e = arr[4];

#define SWAP(x, y)                                                             \
  if ((x) > (y)) {                                                             \
    t = (x);                                                                   \
    (x) = (y);                                                                 \
    (y) = t;                                                                   \
  }

  SWAP(a, b)
  SWAP(d, e)
  SWAP(a, c)
  SWAP(b, c)
  SWAP(a, d)
  SWAP(c, d)
  SWAP(b, e)
  SWAP(b, c)
  SWAP(c, d)

#undef SWAP

  return c;
}

static float median3(const float *arr) {
  float t, a = arr[0], b = arr[1], c = arr[2];

#define SWAP(x, y)                                                             \
  if ((x) > (y)) {                                                             \
    t = (x);                                                                   \
    (x) = (y);                                                                 \
    (y) = t;                                                                   \
  }

  SWAP(a, b)
  SWAP(b, c)

#undef SWAP
  if (b < 0) {
    return c;
  }
  return b;
}

static float median_filter(const float *buf, int len) {
  if (len == 5) {
    return median5(buf);
  } else {
    return median3(buf);
  }
}

float filter_dist(const float *buf, float last_val, float alpha, bool *init,
                  float outlier_thresh, int len) {

  // median
  float res = median_filter(buf, len);

  if (!(*init)) {
    *init = true;
    return res;
  }
  // outlier detection
  if (last_val <= 0 || res <= 0) {
    return res;
  }
  // unreliable in this context
  // else if (fabs(res - last_val) > last_val * outlier_thresh) {
  //   res = last_val;
  // }

  // smoothing with EMA
  return alpha * res + (1 - alpha) * last_val;
}