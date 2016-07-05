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

#include <cstdlib>
#include <cstdint>
#include <algorithm>

#include "perf.h"
#include "controller.h"
#include "fixed.h"


void DQCurrentController::update(
    float out_v_dq_v[2],
    const float i_dq_a[2],
    float angular_velocity_rad_per_s,
    float vbus_v,
    float audio_v
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
    float ed_a, eq_a, vd_v, vq_v, ccd_v, ccq_v, vd_max, vq_max;

    /* Calculate Idq error */
    ed_a = 0.0f - i_dq_a[0];
    eq_a = i_setpoint_a_ - i_dq_a[1];

    /* Limit error -- use conditional instructions to avoid branching */
    if (eq_a > accel_current_limit_a_) {
        eq_a = accel_current_limit_a_;
    }
    if (eq_a < -accel_current_limit_a_) {
        eq_a = -accel_current_limit_a_;
    }

    /* Decouple Idq using feed-forward, and add the integral error */
    ccd_v = integral_vd_v_; // - ls_h_ * angular_velocity_rad_per_s * i_dq_a[1];
    ccq_v = integral_vq_v_; // + ls_h_ * angular_velocity_rad_per_s * i_dq_a[0];

    /*
    Combine integral with proportional terms to find the desired output
    voltages
    */
    vd_v = kp_ * ed_a + ccd_v;
    vq_v = kp_ * eq_a + ccq_v + audio_v;

    /*
    Limit the absolute value of Vdq to vbus * maximum modulation. Give Vd
    priority over Vq if we are limited by voltage, to ensure Id can be kept
    at zero.
    */
    vd_max = std::min(v_limit_v_, vbus_v * 0.95f);
    if (vd_v > vd_max) {
        vd_v = vd_max;
    }
    if (vd_v < -vd_max) {
        vd_v = -vd_max;
    }

    vq_max = __VSQRTF(vd_max * vd_max - vd_v * vd_v);
    if (vq_v > vq_max) {
        vq_v = vq_max;
    }
    if (vq_v < -vq_max) {
        vq_v = -vq_max;
    }

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
