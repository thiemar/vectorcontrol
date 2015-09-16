/*
Copyright (C) 2014-2015 Thiemar Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <cfloat>
#include <cmath>

#if defined(CONFIG_STM32_STM32F302)
    /* STM32F30X-specific CMSIS defs */
    #include "cmsis_stm32f302.h"
#elif defined(CONFIG_STM32_STM32F446)
    /* STM32F44X-specific CMSIS defs */
    #include "cmsis_stm32f446.h"
#endif /* defined(CONFIG_STM32_STM32F302) */

#include "core_cm4.h"


#ifndef M_PI
#define M_PI 3.141592653589793f
#endif


#define RHO_KG_PER_M3 1.225 /* density of air, kg/m^3 */


struct motor_state_t {
    float angular_velocity_rad_per_s;
    float angle_rad; /* 0 .. 2 * pi */
    float i_dq_a[2];
};


struct motor_params_t {
    float rs_r; /* winding resistance in ohms */
    float ls_h; /* winding inductance in ohms */
    float phi_v_s_per_rad; /* speed constant in volt-seconds per radian */

    /* Operating limits */
    float max_current_a; /* RMS current limit in amps */
    float max_voltage_v; /* RMS voltage limit in volts */
    float accel_voltage_v; /* RMS initial open-loop voltage */
    float idle_speed_rad_per_s; /* Speed to spin at when armed but with zero
                                   setpoint, in rad/s */
    float spinup_rate_rad_per_s2; /* Rate at which to increase the motor speed
                                     when starting up. */

    uint32_t num_poles; /* number of poles */
};


struct control_params_t {
    float bandwidth_hz;
    float accel_gain;
    float accel_time_s;

    float prop_cd_kx, prop_cd_k; /* Cd = kx * phi + k */
    float prop_cl_kx, prop_cl_k; /* Cl = kx * phi + k */
    float prop_chord_m;
    float prop_radius_m;
    float prop_geometric_pitch_deg;
    uint32_t prop_num_blades;
};


inline float __attribute__((always_inline)) fast_atan(float x) {
/*
Derived from:
“Efficient approximations for the arctangent function”,
Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
*/
    return float(M_PI / 4.0) * x - x * (std::abs(x) - 1.0f) *
           (0.2447f + 0.0663f * std::abs(x));
}


inline void __attribute__((always_inline))
sin_cos(
    float& sinx,
    float& cosx,
    float x /* x must be in the range [-pi, pi] */
) {
    const float Q = 3.1f;
    const float P = 3.6f;

    float y;

    x *= float(1.0 / M_PI);

    y = x - x * std::abs(x);
    sinx = y * (Q + P * std::abs(y));

    /* Calculate the cosine */
    x += 0.5f;
    if (x > 1.0f) {
        x -= 2.0f;
    }

    y = x - x * std::abs(x);
    cosx = y * (Q + P * std::abs(y));
}


inline float __attribute__((always_inline)) __VSQRTF(float x) {
    float result;
    asm ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (x) );
    return result;
}
