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


#ifndef M_PI
#define M_PI 3.141592653589793f
#endif

/*
fast_XXf functions are copyright (C) 2011 Paul Mineiro
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

inline float __attribute__((optimize("O3")))
fast_pow2f(float p) {
    float offset = (p < 0) ? 1.0f : 0.0f;
    float clipp = (p < -126) ? -126.0f : p;
    int32_t w = (int32_t)clipp;
    float z = clipp - (float)w + offset;
    union { uint32_t i; float f; } v = { (uint32_t)((1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z)) };

    return v.f;
}


inline float __attribute__((optimize("O3")))
fast_expf(float x) {
    return fast_pow2f(x * 1.442695040f);
}


inline float __attribute__((optimize("O3")))
fast_log2f(float x) {
    union { float f; uint32_t i; } vx = { x };
    union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFF) | 0x3f000000 };
    float y = vx.i;
    y *= 1.1920928955078125e-7f;

    return y - 124.22551499f
             - 1.498030302f * mx.f
             - 1.72587999f / (0.3520887068f + mx.f);
}


inline float __attribute__((optimize("O3")))
fast_powf(float x, float p){
  return fast_pow2f(p * fast_log2f(x));
}


inline float __attribute__((optimize("O3")))
fast_sin(float x) {
    const float B = (float)(4.0 / M_PI);
    const float C = (float)(-4.0 / (M_PI * M_PI));
    const float P = 0.225f; /* or 0.218 to minimize relative error */

    float y;
    y = B * x + C * x * std::abs(x);
    return P * (y * std::abs(y) - y) + y;
}


inline float __attribute__((optimize("O3")))
__VSQRTF(float x) {
    float result;
    asm ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (x) );
    return result;
}

#define G_ACCEL_M_PER_S2 9.80665f /* m/s^2 */
#define STANDARD_PRESSURE_PA 101325.0f /* Pa at sea level */
#define STANDARD_TEMP_K (273.15f + 15.0f) /* K */
#define STANDARD_C_M_PER_S 340.27f /* speed of sound at STANDARD_TEMP and
                                      STANDARD_PRESSURE */

/*
Calculate a difference in altitude from differences in pressure at a given
temperature (for standard pressure altitude, use STANDARD_TEMP).

Should not be used for altitudes above 11km, as the standard temperature
lapse rate varies with altitude.

Nominally, both `pref` and `p` are Pa, but since it's a ratio it doesn't
really matter.
*/
inline float __attribute__((optimize("O3")))
altitude_diff_from_pressure_diff(
    float pressure_ref_pa,
    float pressure_pa,
    float temp_k
) {
    return (fast_powf(pressure_ref_pa / pressure_pa, 1.0f / 5.257f) - 1.0f) *
           temp_k * (1.0f / 0.0065f);
}

/*
Calculate the density of air at a given pressure (`p`, in Pa) and temperature
(`temp`, in K).
*/
inline float __attribute__((optimize("O3")))
density_from_pressure_temp(
    float pressure_pa,
    float temp_k
) {
    /* 287.058 is the gas constant for dry air, in J kg^-1 K^-1 */
    return pressure_pa / (287.058f * temp_k);
}

/*
Calculate the airspeed corresponding to a given differential pitot pressure
reading.

`pstatic` is the static pressure in Pa (e.g. from the barometer), `pdynamic`
is the dynamic/impact/differential pressure in Pa, and `temp` is the static
temperature in K.
*/
inline float __attribute__((optimize("O3")))
airspeed_from_pressure_temp(
    float dynamic_pressure_pa,
    float static_pressure_pa,
    float temp_k
) {
    float pressure_ratio;
    pressure_ratio = std::abs(dynamic_pressure_pa / static_pressure_pa);

    return STANDARD_C_M_PER_S *
           __VSQRTF((5.0f / STANDARD_TEMP_K) * temp_k *
                    (fast_powf(pressure_ratio + 1.0f, 2.0f / 7.0f) - 1.0f));
}
