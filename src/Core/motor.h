#pragma once

#include "stdint.h"
#include "tim.h"

typedef struct {
  uint16_t fl, fr, bl, br;
} motors_pwm_s;

#define PWM_MIN 1000U
#define PWM_MAX 2000U

void pwm_set_all(const motors_pwm_s *val);

void pwm_init();

void pwm_deinit();

void pwm_set_pulse_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t us);
