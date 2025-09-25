/*
DSHOT protocol implementation for ESC communication.

Author: Maks Boiar

*/

#include "dshot.h"

#include "tim.h"

typedef enum {
  DSHOT_CMD_MOTOR_STOP,
  DSHOT_CMD_BEEP1,
  DSHOT_CMD_BEEP2,
  DSHOT_CMD_BEEP3,
  DSHOT_CMD_BEEP4,
  DSHOT_CMD_BEEP5,
  DSHOT_CMD_ESC_INFO,
  DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
  DSHOT_CMD_SPIN_DIRECTION_REVERSED,
  DSHOT_CMD_LED0_ON = 22,
  DSHOT_CMD_LED0_OFF = 26

} DSHOT_CMD;

TIM_HandleTypeDef *htim_esc = &htim1;

static uint16_t dshot_pwm_buf0[DSHOT_DMA_BUF_SIZE];
static uint16_t dshot_pwm_buf1[DSHOT_DMA_BUF_SIZE];
static uint16_t dshot_pwm_buf2[DSHOT_DMA_BUF_SIZE];
static uint16_t dshot_pwm_buf3[DSHOT_DMA_BUF_SIZE];

static uint32_t ticks_per_bit = 0;

static uint32_t dshot_type_to_ticks_per_bit(dshot_type_t type) {
  uint32_t timer_freq = HAL_RCC_GetPCLK1Freq();
  return (timer_freq / (htim_esc->Init.Prescaler + 1) +
          (dshotTYPE_TO_HZ(type) / 2)) /
         dshotTYPE_TO_HZ(type);
}

// Initialize DSHOT communication
// @param dshot_type DSHOT frequency
void dshot_init(dshot_type_t type) {
  ticks_per_bit = dshot_type_to_ticks_per_bit(type);
  htim_esc->Init.Period = ticks_per_bit - 1;
}

static void dshot_build_pwm(uint16_t *buf, uint16_t frame) {
  uint16_t ticks_t1h = (uint16_t)((ticks_per_bit * 76) / 100);
  for (size_t i = 0; i < 16; i++) {
    buf[i] = ((frame >> (15 - i)) & 0x1) ? ticks_t1h : ticks_t1h / 2;
  }
}

static uint16_t dshot_pack(uint16_t throttle, uint8_t telem) {
  if (throttle > 2047)
    throttle = 2047;
  uint16_t value = (throttle << 1) | (telem & 0x01);
  uint8_t crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
  return (value << 4) | crc;
}

int dshot_write(uint16_t data, uint8_t telem) {
  uint16_t frame = dshot_pack(data, telem);
  dshot_build_pwm(dshot_pwm_buf0, frame);

  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
  // TODO: burst write
  //   if (HAL_TIM_DMABurst_WriteStart(&htim1, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
  //   (uint32_t*)dshot_pwm_buf0, TIM_DMABURSTLENGTH_4TRANSFERS) != HAL_OK) {
  //     return -1;
  //   }
  if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshot_pwm_buf0,
                            16) != HAL_OK) {
    return -1;
  }
  return 0;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  }
}