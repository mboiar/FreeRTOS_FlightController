#pragma once

#include <stdbool.h>
#include <stdint.h>

#define HCSR04_MAX_ECHO_US 40000U /* 40 ms -> max measurable (~6.8 m) */
#define HCSR04_RISING_TIMEOUT_US                                               \
  5000U                         /* if no rising in 5ms after trig -> no echo */
#define HCSR04_MIN_VALID_US 10U /* ignore extremely short pulses */
#define HCSR04_TRIGGER_PULSE_US 10U /* TRIG high time (10 us) */

typedef struct {
  volatile uint8_t state;       /* 0=IDLE, 1=WAIT_RISING, 2=WAIT_FALLING */
  volatile uint32_t t_start_us; /* timestamp of rising edge */
  volatile uint32_t
      duration_us; /* measured pulse width (us); 0 = none/timeout */
  volatile uint32_t last_trigger_us; /* when triggered this sensor */
  volatile float
      last_distance_cm; /* last computed distance in cm; -1 = no reading */
  uint8_t orientation;
} hcsr04_sensor_t;

void hcsr04_init(hcsr04_sensor_t *sensor);

void hcsr04_reset(hcsr04_sensor_t *sensor);

void hcsr04_trigger();

float duration_to_dist(float dur_s, float temperature);

float filter_dist(const float *buf, float last_val, float alpha, bool *init,
                  float outlier_thresh);
