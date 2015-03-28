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

#include "esc_assert.h"

/* sqrt(3) = sqrt3_mul / sqrt3_div */
#define SQRT3_MUL 28378
#define SQRT3_Q 14


inline uint8_t __attribute__((optimize("O3")))
svm_duty_cycle_from_v_alpha_beta(
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

    phase_oc_ticks[0] = (uint16_t)__USAT(tmpa, 16);
    phase_oc_ticks[1] = (uint16_t)__USAT(tmpb, 16);
    phase_oc_ticks[2] = (uint16_t)__USAT(tmpc, 16);

    return sector;
}
