#ifndef ROBOTMASTER_ARM_MATH_COMPAT_H
#define ROBOTMASTER_ARM_MATH_COMPAT_H

#include <math.h>

/* Only these two CMSIS-DSP calls are used by the copied application code. */
static inline float arm_sin_f32(float x) { return sinf(x); }
static inline float arm_cos_f32(float x) { return cosf(x); }

#endif
