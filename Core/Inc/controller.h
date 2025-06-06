#pragma once

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef attitude_controller(void* eulerAngle, void* attitudeBody, void* ctrlInPitch, void* ctrlInYaw, void* ctrlInRoll);

HAL_StatusTypeDef rate_controller(void* rollRateTarget, void* pitchRateTarget, void* yawRateTarget);

HAL_StatusTypeDef position_controller();


/**
 * @brief Calculate attitude
 * @param argument: Not used
 * @retval 
 */
void calc_attitude(void* eulerAngle, void* attitudeBody);

