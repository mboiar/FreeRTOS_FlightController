#include "motor.h"

void pwm_set_pulse_us(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t us) {
  if (us < PWM_MIN)
    us = PWM_MIN;
  if (us > PWM_MAX)
    us = PWM_MAX;
  __HAL_TIM_SET_COMPARE(htim, channel, us);
}

void pwm_set_all(const motors_pwm_s *val) {
  pwm_set_pulse_us(&htim1, TIM_CHANNEL_1, val->fr);
  pwm_set_pulse_us(&htim1, TIM_CHANNEL_2, val->fl);
  pwm_set_pulse_us(&htim1, TIM_CHANNEL_3, val->bl);
  pwm_set_pulse_us(&htim1, TIM_CHANNEL_4, val->br);
}

void pwm_init() {
  htim1.Instance->PSC = 99;
  htim1.Instance->ARR = 19999;
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
}

void pwm_deinit() {
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
}
