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

#include "fixed.h"
#include "esc_assert.h"


class DQCurrentController {
    /* Integral D and Q current errors */
    float integral_vd_v_;
    float integral_vq_v_;

    /* Total current setpoint */
    float i_setpoint_a_;

    /* PI coefficients for the current controllers. */
    float kp_;
    float ki_;

    /* Feed-forward coefficients */
    float ls_h_;

    /*
    Motor voltage limit (will be constrained to the lower of this and
    0.95 * Vbus)
    */
    float v_limit_v_;
    float accel_current_limit_a_;

public:
    DQCurrentController(void):
        integral_vd_v_(0.0f),
        integral_vq_v_(0.0f),
        i_setpoint_a_(0.0f),
        kp_(0.0f),
        ki_(0.0f),
        ls_h_(0.0f),
        v_limit_v_(0.125f),
        accel_current_limit_a_(0.0f)
    {}

    void reset_state(void) {
        i_setpoint_a_ = 0.0f;
        integral_vd_v_ = 0.0f;
        integral_vq_v_ = 0.0f;
    }

    void set_setpoint(float s_current_a) {
        i_setpoint_a_ = s_current_a;
    }

    void set_v_limit_v(float limit) {
        v_limit_v_ = limit;
    }

    void set_params(
        const struct motor_params_t& params,
        const struct control_params_t& control_params,
        float t_s
    ) volatile {
        float bandwidth_rad_per_s;

        ls_h_ = params.ls_h;

        /*
        Convert control bandwidth to rad/s. Current controller bandwidth will
        be set to 10x this value.
        */
        bandwidth_rad_per_s = float(2.0 * M_PI) * control_params.bandwidth_hz;

        /*
        Parameter selection described in:

        Harnefors, L. and Nee, H-P. (1998)
        "Model-Based Current Control of AC Machines Using the Internal Model
        Control Method",

        http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=658735

        Equation (19):

        Kp = wc * Ls
        Ti = Ls / Rs
        Ki = 1 / Ti

        Scale Ki by t_s to avoid an extra multiplication each timestep.
        */
        kp_ = ls_h_ * bandwidth_rad_per_s * 10.0f;
        ki_ = t_s * params.rs_r / ls_h_;

        /* Set modulation limit */
        v_limit_v_ = params.max_voltage_v;

        accel_current_limit_a_ = params.accel_voltage_v / params.rs_r;
    }

    void update(
        float out_v_dq_v[2],
        const float i_dq_a[2],
        float angular_velocity_frac_per_timestep,
        float vbus_v,
        float audio_v
    );
};

