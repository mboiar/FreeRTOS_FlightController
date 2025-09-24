#include "startup_task.h"
#include "task.h"
#include "w25q64.h"
#include "logging_task.h"

static char DefaultTaskLog[BUFFER_SIZE] = {0};

void StartupTask(void *argument) {
  // uint32_t PreviousWakeTime = osKernelSysTick();
  // uint32_t OctaveFour[7] = {262, 294, 330, 349, 392, 440, 493}; // Octave_4
  // Frequencies (C4->B4)
  uint8_t NoteIndex = 0;

  // Tone cur_tone;
  static UBaseType_t blocked = 1;

  device_info device_info;
  w25q64_device_info(&device_info);

  status_registers status_bits;
  w25q64_status(&status_bits);

  // memcpy(DefaultTaskLog, "Default task is called here   \r\n", 33);

  bool melody_completed = false;
  // vTaskSuspend(NULL);
  uint32_t ulNotifiedValue = 0;
  BaseType_t xResult;
  BaseType_t xStatus;
  mavlink_message_t msg;

  /* Infinite loop */
  for (;;) {
    // memset(DefaultTaskLog, 0, BUFFER_SIZE);
    mavlink_log(MAV_SEVERITY_INFO, &msg, DefaultTaskLog, "[Default] Called");
    LOG_INFO(&msg);
    if (xStatus != pdPASS) {
      // handle queue fail
    }
    // if (blocked) {
    //   // if (ulTaskNotifyTake(pdFALSE, portMAX_DELAY) == pdTRUE) {
    //   xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue,
    //   portMAX_DELAY); if (xResult == pdPASS) {
    //     if ((ulNotifiedValue & 0x01) != 0) {
    //       blocked = 0;
    //     }
    //   }
    // } else {
    //   // if (ulTaskNotifyTake(pdFALSE, 0)) {
    //   xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, 0);
    //   if (xResult == pdPASS) {
    //     if ((ulNotifiedValue & 0x01) != 0) {
    //       blocked = 1;
    //       TIM1->CCR1 = 0;
    //       continue;
    //     }
    //   }
    // }
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if (NoteIndex == 25) {
      //   NoteIndex = 0;
      melody_completed = true;
      // vTaskDelay(pdMS_TO_TICKS(2000));
      vTaskSuspend(NULL);
    } else {
      int freq = 330; // melody[NoteIndex];
      int dur = 4;    // durations[NoteIndex];
      // cur_tone = canon_melody[NoteIndex];
      if (freq > 0) {
        TIM1->ARR = (1000000UL / freq) - 1; // Set The PWM Frequency
        TIM1->CCR1 = (TIM1->ARR >> 1);      // Set Duty Cycle 50%
      } else {
        TIM1->CCR1 = 0;
      }
      NoteIndex++;
      vTaskDelay(pdMS_TO_TICKS(1000 / dur));
    }
  }
}