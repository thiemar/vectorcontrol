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
    float v_limit_;
    float accel_current_limit_a_;

public:
    DQCurrentController(void):
        integral_vd_v_(0.0f),
        integral_vq_v_(0.0f),
        i_setpoint_a_(0.0f),
        kp_(0.0f),
        ki_(0.0f),
        ls_h_(0.0f),
        v_limit_(0.1f),
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

    void set_params(
        const struct motor_params_t& params,
        const struct control_params_t& control_params,
        float t_s
    ) {
        float bandwidth_rad_per_s;

        ls_h_ = params.ls_h;

        /*
        Convert control bandwidth to rad/s. Current controller bandwidth will
        be set to 10x this value.
        */
        bandwidth_rad_per_s = 2.0f * (float)M_PI *
                              control_params.bandwidth_hz;

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
        v_limit_ = params.max_voltage_v;

        accel_current_limit_a_ = params.max_current_a;
    }

    void update(
        float out_v_dq_v[2],
        const float i_dq_a[2],
        float angular_velocity_frac_per_timestep,
        float vbus_v,
        float audio_v
    );
};


class SpeedController {
protected:
    float integral_error_a_;
    float setpoint_rad_per_s_;

    float kp_;
    float ki_;
    float kp_inv_;

    float min_speed_rad_per_s_;
    float current_limit_a_;
    float accel_current_limit_a_;
    float speed_limit_rad_per_s_;

public:
    SpeedController():
        integral_error_a_(0.0f),
        setpoint_rad_per_s_(0.0f),
        kp_(0.0f),
        ki_(0.0f),
        kp_inv_(0.0f),
        min_speed_rad_per_s_(0.0f),
        current_limit_a_(0.0f),
        accel_current_limit_a_(0.0f),
        speed_limit_rad_per_s_(0.0f)
    {}

    void reset_state() {
        integral_error_a_ = 0.0f;
        setpoint_rad_per_s_ = 0.0f;
    }

    void set_setpoint(float setpoint) {
        setpoint_rad_per_s_ = setpoint;
    }

    void set_current_limit_a(float limit) {
        current_limit_a_ = limit;
    }

    void set_params(
        const struct motor_params_t& motor_params,
        const struct control_params_t& control_params,
        float t_s
    ) {
        kp_ = 0.01f * control_params.max_accel_torque_a *
              control_params.accel_gain;
        ki_ = t_s / control_params.accel_time_s;

        min_speed_rad_per_s_ = motor_params.min_speed_rad_per_s;
        speed_limit_rad_per_s_ = motor_params.max_speed_rad_per_s;
        current_limit_a_ = motor_params.max_current_a;
        accel_current_limit_a_ = control_params.max_accel_torque_a;
    }

    float __attribute__((optimize("O3")))
    update(const struct motor_state_t& state) {
        float error_rad_per_s, accel_torque_a, result_a, out_a;

        error_rad_per_s = setpoint_rad_per_s_ -
                          state.angular_velocity_rad_per_s;
        if (error_rad_per_s > min_speed_rad_per_s_) {
            error_rad_per_s = min_speed_rad_per_s_;
        } else if (error_rad_per_s < -min_speed_rad_per_s_) {
            error_rad_per_s = -min_speed_rad_per_s_;
        }

        /*
        Determine acceleration torque, and limit to the configured maximum
        */
        accel_torque_a = kp_ * error_rad_per_s;

        if (accel_torque_a > accel_current_limit_a_) {
            accel_torque_a = accel_current_limit_a_;
        } else if (accel_torque_a < -accel_current_limit_a_) {
            accel_torque_a = -accel_current_limit_a_;
        }

        out_a = accel_torque_a + integral_error_a_;

        /* Limit output to maximum current */
        if (out_a > current_limit_a_) {
            result_a = current_limit_a_;
        } else if (out_a < -current_limit_a_) {
            result_a = -current_limit_a_;
        } else {
            result_a = out_a;
        }

        /*
        Limit integral term accumulation when the output current is saturated
        (or when it's lagging behind the setpoint).
        */
        if (std::abs(state.angular_velocity_rad_per_s) > min_speed_rad_per_s_ &&
                state.angular_velocity_rad_per_s * setpoint_rad_per_s_ > 0.0f) {
            integral_error_a_ += ki_ * (result_a - integral_error_a_);
        } else {
            integral_error_a_ += ki_ * (result_a - integral_error_a_) * 1e-3f;
        }

        return result_a;
    }
};


class AlignmentController {
    /* Ia PI coefficients and state */
    float integral_va_v_;
    float v_kp_;
    float v_ki_;

    /* Position controller coefficients and state */
    float integral_ia_a_;
    float i_kp_;
    float i_ki_;

    /* Position controller output (current setpoint) */
    float i_setpoint_a_;

    /* Physical parameters */
    float v_limit_;
    float i_limit_;

public:
    AlignmentController(void):
        integral_va_v_(0.0f),
        v_kp_(0.0f),
        v_ki_(0.0f),
        integral_ia_a_(0.0f),
        i_kp_(0.0f),
        i_ki_(0.0f),
        i_setpoint_a_(0.0f),
        v_limit_(0.0f),
        i_limit_(0.0f)
    {}

    void reset_state(void) {
        integral_va_v_ = 0.0f;
        integral_ia_a_ = 0.0f;
        i_setpoint_a_ = 0.0f;
    }

    void set_params(
        const struct motor_params_t& params,
        const struct control_params_t& control_params,
        float t_s
    ) {
        float bandwidth_rad_per_s;

        /*
        Current controller bandwidth is the same as for the main current
        controller
        */
        bandwidth_rad_per_s = 2.0f * (float)M_PI *
                              control_params.bandwidth_hz * 10.0f;
        v_kp_ = params.ls_h * bandwidth_rad_per_s;
        v_ki_ = t_s * params.rs_r / params.ls_h;

        /*
        Gain is 10% of max current per radian position error.
        Integral time is 0.02 s.
        */
        i_kp_ = params.max_current_a * 0.1f;
        i_ki_ = t_s;

        /* Set modulation and current limit */
        v_limit_ = params.max_voltage_v;
        i_limit_ = params.max_current_a;
    }

    void __attribute__((optimize("O3")))
    update(
        float out_v_ab_v[2],
        const float i_ab_a[2],
        float angle_rad,
        float vbus_v
    ) {
        float v_max, etheta_rad, ea_a, va_v;

        /*
        Determine how much current the D axis needs to align properly. We take
        absolute error as the input because we don't care about the direction
        of the error -- as long as enough current is flowing through D the
        rotor will converge on the initial position.
        */
        etheta_rad = std::abs(angle_rad > (float)M_PI ?
                              angle_rad - (float)M_PI * 2.0f : angle_rad);
        i_setpoint_a_ = i_kp_ * etheta_rad + integral_ia_a_;

        if (i_setpoint_a_ > i_limit_) {
            i_setpoint_a_ = i_limit_;
        }

        integral_ia_a_ += i_ki_ * (i_setpoint_a_ - integral_ia_a_);

        /*
        Single-component current controller to run enough current in the D
        direction to achieve rotor alignment.
        */
        v_max = std::min(v_limit_, vbus_v * 0.95f);

        ea_a = i_setpoint_a_ - i_ab_a[0];
        va_v = v_kp_ * ea_a + integral_va_v_;

        if (va_v > v_max) {
            va_v = v_max;
        }

        integral_va_v_ += v_ki_ * (va_v - integral_va_v_);

        out_v_ab_v[0] = va_v;
        out_v_ab_v[1] = 0.0f;
    }
};
