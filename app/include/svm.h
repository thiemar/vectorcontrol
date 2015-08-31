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

#include "esc_assert.h"

/* sqrt(3) = sqrt3_mul / sqrt3_div */
#define SQRT3_MUL 28378
#define SQRT3_Q 14


inline uint8_t svm_duty_cycle_from_v_alpha_beta(
    uint16_t phase_oc_ticks[3],
    int16_t v_frac_alpha_q1p15,
    int16_t v_frac_beta_q1p15,
    uint16_t pwm_period_ticks
) {
    int32_t x, y, z, u_alpha, u_beta, tmpa, tmpb, tmpc;
    int32_t quarter_t;
    uint32_t double_period_t;
    uint8_t sector;

    quarter_t = pwm_period_ticks >> 2;
    double_period_t = pwm_period_ticks << 1u;

    /* See ST UM1052, pp. 30-32. */
    u_alpha = v_frac_alpha_q1p15 * ((double_period_t * SQRT3_MUL) >> SQRT3_Q);
    u_beta = v_frac_beta_q1p15 * double_period_t;

    x = u_beta;
    y = (u_beta + u_alpha) >> 1;
    z = (u_beta - u_alpha) >> 1;

    /* Sector calculation from x, y, z */
    if (y < 0) {
        if (z < 0) {
            sector = 5;
        } else if (x <= 0) {
            sector = 4;
        } else {
            sector = 3;
        }
    } else {
        if (z >= 0) {
            sector = 2;
        } else if (x <= 0) {
            sector = 6;
        } else {
            sector = 1;
        }
    }

    switch (sector) {
        case 1:
        case 4:
            tmpa = quarter_t + ((x - z) >> 18);
            tmpb = tmpa + (z >> 17);
            tmpc = tmpb - (x >> 17);
            break;
        case 2:
        case 5:
            tmpa = quarter_t + ((y - z) >> 18);
            tmpb = tmpa + (z >> 17);
            tmpc = tmpa - (y >> 17);
            break;
        case 3:
        case 6:
            /* Note different order of calculation from above two cases */
            tmpa = quarter_t + ((y - x) >> 18);
            tmpc = tmpa - (y >> 17);
            tmpb = tmpc + (x >> 17);
            break;
        default:
            esc_assert(false);
            break;
    }

    phase_oc_ticks[0] = uint16_t(__USAT(tmpa, 16u));
    phase_oc_ticks[1] = uint16_t(__USAT(tmpb, 16u));
    phase_oc_ticks[2] = uint16_t(__USAT(tmpc, 16u));

    return sector;
}
