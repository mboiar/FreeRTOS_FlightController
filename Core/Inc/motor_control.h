/*
 * motor_control.h
 *
 *  Created on: Apr 25, 2025
 *      Author: boiar
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32f4xx.h"
#include "tim.h"

typedef struct {
    uint32_t outLF, outRF, outLB, outRB;
} motor_ctrl;

void motor_set_ctrl(const motor_ctrl* ctrl) {
    TIM3->CCR1 = ctrl->outLF;
    TIM3->CCR2 = ctrl->outRF;
    TIM3->CCR3 = ctrl->outLB;
    TIM3->CCR4 = ctrl->outRB;
}

void motor_stop() {
    motor_ctrl ctrl = {0, 0, 0, 0};
    motor_set_ctrl(&ctrl);
}

#endif /* INC_MOTOR_CONTROL_H_ */
