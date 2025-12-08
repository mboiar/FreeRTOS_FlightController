#include <pid.h>


void pid_init(pid_s *pid, float Kp, float Ki, float Kd, float out_min, float out_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_err = 0;
    pid->integral = 0;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float pid_compute(pid_s *pid, float meas, float sp, float dt) {
    float err = sp - meas;

    pid->integral += err * dt;
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    float deriv = (err - pid->prev_err) / dt;
    float out = pid->Kp*err + pid->Ki*pid->integral + pid->Kd*deriv;
    pid->prev_err = err;

    if (out > pid->out_max) return pid->out_max;
    if (out < pid->out_min) return pid->out_min;

    return out;
}