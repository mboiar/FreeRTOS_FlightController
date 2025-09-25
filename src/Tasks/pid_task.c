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
  }
}