#include "Tasks.h"

typedef struct {
  uint32_t ts;
  uint16_t sensor_id;
  GPIO_PinState is_rising;
} EdgeEvent;

static hcsr04_sensor_t dist_sensors[HCSR04_SENSOR_COUNT];
static float dist_buf[HCSR04_SENSOR_COUNT][HCSR04_BUFFER_LEN];
static int dist_meas_cnt = 0;
static bool dist_filter_init[HCSR04_SENSOR_COUNT] = {0};

bool distance_sensor_ready_all = false;

static GPIO_TypeDef *HCSR04_ECHO_PORT[HCSR04_SENSOR_COUNT] = {
    GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA};

static uint16_t HCSR04_ECHO_PIN[HCSR04_SENSOR_COUNT] = {
    GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10, GPIO_PIN_13, GPIO_PIN_15, GPIO_PIN_12};

static TickType_t last_dist_tick;
static uint32_t cur_tick, notif;

void handleDistEvent(const EdgeEvent *ev);

void StartupTask(void *argument) {
  mavlink_message_t msg;
  EdgeEvent event;

  for (int i = 0; i < HCSR04_SENSOR_COUNT; i++) {
    hcsr04_init(&dist_sensors[i]);
  }
  hcsr04_trigger();
  last_dist_tick = xTaskGetTickCount();

  for (;;) {
    if (xQueueReceive(distQueue, &event, pdMS_TO_TICKS(1))) {
      handleDistEvent(&event);
    }
    if (xTaskNotifyWait(pdFALSE, 0x01, &notif, 0) == pdTRUE) {
      notif &= ~0x01;
      cur_tick = get_time_since_boot_us();

      if (dist_meas_cnt % HCSR04_BUFFER_LEN ==
          HCSR04_BUFFER_LEN - 1) { // filter and report on buffer full
        for (uint8_t i = 0; i < HCSR04_SENSOR_COUNT; i++) {
          dist_sensors[i].last_distance_cm =
              filter_dist(dist_buf[i], dist_sensors[i].last_distance_cm, 0.8,
                          &(dist_filter_init[i]), 2, HCSR04_BUFFER_LEN);

          mavlink_msg_command_long_pack(
              1, MAV_COMP_ID_AUTOPILOT1, &msg, 2, MAV_COMP_ID_ONBOARD_COMPUTER,
              5000, 0, dist_sensors[0].last_distance_cm,
              dist_sensors[1].last_distance_cm,
              dist_sensors[2].last_distance_cm,
              dist_sensors[3].last_distance_cm,
              dist_sensors[4].last_distance_cm,
              dist_sensors[5].last_distance_cm, cur_tick);
          comm_tx_send(&msg);
        }
      }
      dist_meas_cnt++;
      for (uint8_t i = 0; i < HCSR04_SENSOR_COUNT; i++) {
        hcsr04_reset(&dist_sensors[i]);
      }
      cur_tick = TIM3->CNT;
      hcsr04_trigger();
      if (xTaskNotifyWait(pdFALSE, 0x02, &notif, portMAX_DELAY) == pdTRUE &&
          (notif & 0x02)) {
        notif &= ~0x02;
      }
      last_dist_tick = TIM3->CNT;
    }
  }
}

void handleDistEvent(const EdgeEvent *ev) {
  float dur;

  if (ev->is_rising == GPIO_PIN_SET) {
    /* Rising edge */
    dist_sensors[ev->sensor_id].t_start_us = ev->ts;
    dist_sensors[ev->sensor_id].state = 2; /* WAIT_FALLING */
  } else {
    /* Falling edge */
    if (dist_sensors[ev->sensor_id].state == 2 &&
        dist_sensors[ev->sensor_id].t_start_us != 0) {
      dur = (float)(get_time_since_boot_us() -
                    dist_sensors[ev->sensor_id].t_start_us);
      dist_sensors[ev->sensor_id].duration_us = dur;

      // if (dur >= HCSR04_MIN_VALID_US && dur <= HCSR04_MAX_ECHO_US) {
      dist_buf[ev->sensor_id][dist_meas_cnt % HCSR04_BUFFER_LEN] =
          duration_to_dist(dur, 22.2);
      // } else {
      // dist_buf[i][dist_meas_cnt % HCSR04_BUFFER_LEN] = -1.0f; /* invalid
      // */
      // }
    } else {
      dist_sensors[ev->sensor_id].duration_us = 0;
      dist_buf[ev->sensor_id][dist_meas_cnt % HCSR04_BUFFER_LEN] = -1.0f;
    }
    dist_sensors[ev->sensor_id].state = 0; /* IDLE after measurement */
  }
}

void DistanceSensor_RxCpltCallback(uint16_t GPIO_Pin) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  EdgeEvent ev;
  int i;
  switch (GPIO_Pin) {
  case GPIO_PIN_1:
    i = 0;
    break;
  case GPIO_PIN_2:
    i = 1;
    break;
  case GPIO_PIN_10:
    i = 2;
    break;
  case GPIO_PIN_12:
    i = 3;
    break;
  case GPIO_PIN_13:
    i = 4;
    break;
  case GPIO_PIN_15:
    i = 5;
    break;
  default:
    i = -1;
    break;
  }
  if (i > -1) {
    GPIO_PinState level =
        HAL_GPIO_ReadPin(HCSR04_ECHO_PORT[i], HCSR04_ECHO_PIN[i]);
    ev.is_rising = level;
    ev.ts = get_time_since_boot_us();
    ev.sensor_id = i;
    if (distQueue != NULL) {
      xQueueSendFromISR(distQueue, &ev, &xHigherPriorityTaskWoken);
    }
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}