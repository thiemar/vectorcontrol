/*
Copyright (c) 2014 - 2015 by Thiemar Pty Ltd

This file is part of vectorcontrol.

vectorcontrol is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

vectorcontrol is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
vectorcontrol. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <cfloat>
#include <cmath>


#ifndef M_PI
#define M_PI 3.141592653589793f
#endif


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
    float max_voltage_v; /* RMS voltage limit in volts */
    float max_current_a; /* RMS current limit in amps */
    float max_speed_rad_per_s; /* Electrical angular velocity limit in rad/s */
    float startup_current_a; /* RMS current limit during start-up */
    float startup_speed_rad_per_s; /* Open-loop angular velocity in rad/s */

    uint32_t num_poles; /* number of poles */
};


struct control_params_t {
    float gain_a_s_per_rad;
    float bandwidth_hz;
    float max_accel_rad_per_s2;
};


#define APPROXIMATE_SIN_COS
#define APPROXIMATE_EXP

#pragma GCC optimize("O3")
inline float fast_expf(float x) {
#ifdef APPROXIMATE_EXP
/*
fast_expf is copyright (C) 2011 Paul Mineiro
All rights reserved.

Redistribution and use in source and binary forms, with
or without modification, are permitted provided that the
following conditions are met:

    * Redistributions of source code must retain the
    above copyright notice, this list of conditions and
    the following disclaimer.

    * Redistributions in binary form must reproduce the
    above copyright notice, this list of conditions and
    the following disclaimer in the documentation and/or
    other materials provided with the distribution.

    * Neither the name of Paul Mineiro nor the names
    of other contributors may be used to endorse or promote
    products derived from this software without specific
    prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Contact: Paul Mineiro <paul@mineiro.com>
*/
  float p = x * 1.442695040f;
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int32_t w = (int32_t)clipp;
  float z = clipp - (float)w + offset;
  union { uint32_t i; float f; } v = { (uint32_t)((1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z)) };

  return v.f;
#else
    return expf(x);
#endif
}


#pragma GCC optimize("O3")
inline void sin_cos(
    float& sinx,
    float& cosx,
    float x
) {
#ifdef APPROXIMATE_SIN_COS
    const float B = (float)(4.0 / M_PI);
    const float C = (float)(-4.0 / (M_PI * M_PI));
    const float P = 0.225f; /* or 0.218 to minimize relative error */

    float y;

    /* Move x to the range [-pi, pi] */
    if (x > (float)M_PI) {
        x -= (float)(2.0 * M_PI);
    }

    y = B * x + C * x * std::abs(x);
    sinx = P * (y * std::abs(y) - y) + y;

    /* Calculate the cosine */
    x += 0.5f * (float)M_PI;
    if (x > (float)M_PI) {
        x -= (float)(2.0 * M_PI);
    }

    y = B * x + C * x * std::abs(x);
    cosx = P * (y * std::abs(y) - y) + y;

#else
    sinx = sinf(x);
    cosx = cosf(x);
#endif
}


#ifndef STM32F30X
    inline static int32_t __SSAT(int32_t x, uint32_t y) {
        int32_t maxv;

        maxv = 1 << (y - 1);

        if (x > maxv - 1) {
            x = maxv - 1;
        } else if (x < -maxv) {
            x = -maxv;
        }
        return x;
    }

    inline static uint32_t __USAT(uint32_t x, uint32_t y) {
        uint32_t maxv;

        maxv = 1u << y;

        if (x > maxv - 1u) {
            x = maxv - 1u;
        }
        return x;
    }

    inline static float __VSQRTF(float x) {
        return sqrtf(x);
    }
#else
    __attribute__( ( always_inline ) ) inline static float __VSQRTF(float x) {
        float result;
        asm ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (x) );
        return result;
    }
#endif
