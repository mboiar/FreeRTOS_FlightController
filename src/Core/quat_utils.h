#pragma once

#include "arm_math.h"
#include "dsp/fast_math_functions.h"
#include "dsp/matrix_functions.h"

// Rotate vector by a quaternion.
void quat_rotate_vec(float dst[3], const float quat[4], const float vec[3]);
