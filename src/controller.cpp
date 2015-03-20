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

#include <cstdlib>
#include <cstdint>
#include <algorithm>

#include "perf.h"
#include "controller.h"
#include "fixed.h"

#pragma GCC optimize("O3")
void DQCurrentController::update(
    float out_v_dq_v[2],
    const float i_dq_a[2],
    float angular_velocity_rad_per_s,
    float vbus_v
) {
    /*
    Current controller described in:

    Harnefors, L. and Nee, H-P. (1998)
    "Model-Based Current Control of AC Machines Using the Internal Model
    Control Method",

    http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=658735

    This is an implementation of the Diagonal Internal Model Control approach
    (equation 20) with anti-windup during inverter saturation
    (equations 42-47).

    The code below follows the pseudocode listing in Appendix C of that paper.
    */
    float ed_a, eq_a, vd_v, vq_v, ccd_v, ccq_v, v_max, v_mag, scale;

    v_max = std::min(v_limit_, vbus_v * 0.95f);

    /* Calculate Idq error */
    ed_a = 0.0f - i_dq_a[0];
    eq_a = i_setpoint_a_ - i_dq_a[1];

    /* Limit error */
    if (eq_a > accel_current_limit_a_) {
        eq_a = accel_current_limit_a_;
    } else if (eq_a < -accel_current_limit_a_) {
        eq_a = -accel_current_limit_a_;
    }

    /* Decouple Idq using feed-forward, and add the integral error */
    ccd_v = integral_vd_v_ - ls_h_ * angular_velocity_rad_per_s * i_dq_a[1];
    ccq_v = integral_vq_v_ + ls_h_ * angular_velocity_rad_per_s * i_dq_a[0];

    /*
    Combine integral with proportional terms to find the desired output
    voltages
    */
    vd_v = kp_ * ed_a + ccd_v;
    vq_v = kp_ * eq_a + ccq_v;

    /*
    Limit the absolute value of Vdq to vbus * maximum modulation.
    */
    v_mag = __VSQRTF(vd_v * vd_v + vq_v * vq_v);
    if (v_mag > v_max) {
        scale = v_max / v_mag;
    } else {
        scale = 1.0f;
    }

    vd_v *= scale;
    vq_v *= scale;

    /*
    Anti-windup calculation for the integral term; if desired |Vdq| is greater
    than Vmax, reduce the amount of error accumulated.
    */
    integral_vd_v_ += ki_ * (vd_v - ccd_v);
    integral_vq_v_ += ki_ * (vq_v - ccq_v);

    /* Output Vdq */
    out_v_dq_v[0] = vd_v;
    out_v_dq_v[1] = vq_v;
}
