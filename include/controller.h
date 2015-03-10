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
    float v_limit_;

public:
    DQCurrentController():
        integral_vd_v_(0.0f),
        integral_vq_v_(0.0f),
        i_setpoint_a_(0.0f),
        kp_(0.0f),
        ki_(0.0f),
        ls_h_(0.0f),
        v_limit_(0.1f)
    {}

    void reset_state() {
        i_setpoint_a_ = 0.0f;
        integral_vd_v_ = 0.0f;
        integral_vq_v_ = 0.0f;
    }

    void set_setpoint(float s_current_a) {
        i_setpoint_a_ = s_current_a;
    }

    void set_params(
        const struct motor_params_t& params,
        const struct control_params_t& control_params,
        float t_s
    ) {
        float bandwidth_rad_per_s;

        ls_h_ = params.ls_h;

        /* Current control bandwidth is always 10x speed control bandwidth */
        bandwidth_rad_per_s = 2.0f * (float)M_PI *
                              control_params.bandwidth_hz * 10.0f;

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
        kp_ = ls_h_ * bandwidth_rad_per_s;
        ki_ = t_s * params.rs_r / ls_h_;

        /* Set modulation limit */
        v_limit_ = params.max_voltage_v;
    }

    void update(
        float out_v_dq_frac[2],
        const float i_dq_frac[2],
        float angular_velocity_frac_per_timestep,
        float vbus_frac
    );
};

class SpeedController {
protected:
    float integral_error_a_;
    float setpoint_rad_per_s_;

    float kp_;
    float ki_;
    float kp_inv_;

    float current_limit_a_;
    float speed_limit_rad_per_s_;

    /* Maximum setpoint slew rate */
    float setpoint_slew_rate_;
    float setpoint_slew_mul_;

    /* Maximum current limit slew rate */
    float current_limit_slew_rate_;

public:
    SpeedController():
        integral_error_a_(0.0f),
        setpoint_rad_per_s_(0.0f),
        kp_(0.0f),
        ki_(0.0f),
        kp_inv_(0.0f),
        current_limit_a_(0.0f),
        speed_limit_rad_per_s_(0.0f),
        setpoint_slew_rate_(0.0f),
        setpoint_slew_mul_(0.0f),
        current_limit_slew_rate_(0.0f)
    {}

    void reset_state() {
        integral_error_a_ = 0.0f;
        setpoint_rad_per_s_ = 0.0f;
        setpoint_slew_mul_ = 0.0f;
    }

    #pragma GCC optimize("03")
    void set_setpoint(float setpoint) {
        float slew_rate;
        slew_rate = setpoint_slew_rate_ * setpoint_slew_mul_;

        if (setpoint > setpoint_rad_per_s_ + slew_rate) {
            setpoint_rad_per_s_ += slew_rate;
        } else if (setpoint < setpoint_rad_per_s_ - slew_rate) {
            setpoint_rad_per_s_ -= slew_rate;
        } else {
            setpoint_rad_per_s_ = setpoint;
        }
    }

    void set_current_limit(float limit) {
        /* Output current limit */
        if (limit > current_limit_a_ + current_limit_slew_rate_) {
            current_limit_a_ += current_limit_slew_rate_;
        } else if (limit < current_limit_a_ - current_limit_slew_rate_) {
            current_limit_a_ -= current_limit_slew_rate_;
        } else {
            current_limit_a_ = limit;
        }
    }

    void set_params(
        const struct motor_params_t& motor_params,
        const struct control_params_t& control_params,
        float t_s
    ) {
        kp_ = control_params.gain_a_s_per_rad;
        kp_inv_ = 1.0f / kp_;
        ki_ = control_params.gain_a_s_per_rad *
              2.0f * (float)M_PI * control_params.bandwidth_hz * t_s;

        /*
        Maximum setpoint slew rate is from zero to max speed over a period
        equal to the controller integral time. Scale by timestep to avoid
        needing to multiply later.
        */
        setpoint_slew_rate_ = control_params.max_accel_rad_per_s2 * t_s;
        current_limit_slew_rate_ = 0.05f * motor_params.max_current_a *
                                   control_params.bandwidth_hz * t_s;

        speed_limit_rad_per_s_ = motor_params.max_speed_rad_per_s;
    }

    #pragma GCC optimize("O3")
    float update(const struct motor_state_t& state) {
        float error_rad_per_s, result_a, out_a, windup_correction;

        error_rad_per_s = setpoint_rad_per_s_ -
                          state.angular_velocity_rad_per_s;

        out_a = kp_ * error_rad_per_s + integral_error_a_;

        /* Limit output to maximum current */
        if (std::abs(out_a) < current_limit_a_) {
            result_a = out_a;
        } else if (out_a >= 0.0f) {
            result_a = current_limit_a_;
        } else if (out_a < 0.0f) {
            result_a = -current_limit_a_;
        } else {
            result_a = 0.0f;
        }

        /*
        Limit integral term accumulation when the output current is saturated
        (or when it's lagging behind the setpoint).
        */
        windup_correction = result_a - out_a;
        if (error_rad_per_s > 0.0f) {
            windup_correction = std::min(windup_correction, 0.0f);
        } else if (error_rad_per_s < 0.0f) {
            windup_correction = std::max(windup_correction, 0.0f);
        }
        integral_error_a_ +=
            ki_ * (error_rad_per_s + windup_correction * kp_inv_);

        setpoint_slew_mul_ = state.angular_velocity_rad_per_s /
                             (speed_limit_rad_per_s_ * 0.25f);
        setpoint_slew_mul_ = std::min(1.0f, 0.02f + setpoint_slew_mul_);

        return result_a;
    }
};
