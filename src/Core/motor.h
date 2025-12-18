#pragma once

#include "stdint.h"

typedef struct {
  uint16_t fl, fr, bl, br;
} motors_pwm_s;

#define PWM_MIN 1000U
#define PWM_MAX 2000U

void pwm_set_all(const motors_pwm_s *val);

void pwm_init();

void pwm_deinit();
