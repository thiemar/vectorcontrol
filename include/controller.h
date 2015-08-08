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
    /*
    Implements standard PI speed control, as well as the aerodynamic power
    controller described in:

    "Aerodynamic Power Control for Multirotor Aerial Vehicles"
    Bangura, M.; Lim, H.; Jin Kim, H.; Mahony, R. (2014)
    */
protected:
    bool control_power_;

    float integral_error_a_;
    float speed_setpoint_rad_per_s_;

    float power_setpoint_w_;

    float speed_kp_;
    float speed_ki_;

    float power_kp_;
    float power_ki_;

    float ir_;  /* Rotor inertia in kg * m^2*/
    float kq0_; /* Torque constant in N*m / A */

    float current_limit_a_;
    float accel_current_limit_a_;

public:
    SpeedController():
        control_power_(false),
        integral_error_a_(0.0f),
        speed_setpoint_rad_per_s_(0.0f),
        power_setpoint_w_(0.0f),
        speed_kp_(0.0f),
        speed_ki_(0.0f),
        power_kp_(0.0f),
        power_ki_(0.0f),
        ir_(0.0f),
        kq0_(0.0f),
        current_limit_a_(0.0f),
        accel_current_limit_a_(0.0f)
    {}

    void reset_state() {
        integral_error_a_ = 0.0f;
        speed_setpoint_rad_per_s_ = 0.0f;
        power_setpoint_w_ = 0.0f;
    }

    void set_speed_setpoint(float setpoint) {
        speed_setpoint_rad_per_s_ = setpoint;
        control_power_ = false;
    }

    void set_power_setpoint(float setpoint) {
        power_setpoint_w_ = setpoint;
        control_power_ = true;
    }

    void set_current_limit_a(float limit) {
        current_limit_a_ = limit;
    }

    void set_params(
        const struct motor_params_t& motor_params,
        const struct control_params_t& control_params,
        float t_s
    ) {
        speed_kp_ = 0.01f * control_params.max_accel_torque_a *
                    control_params.accel_gain;
        speed_ki_ = t_s / control_params.accel_time_s;

        power_kp_ = control_params.max_accel_torque_a *
                    control_params.accel_gain;
        power_ki_ = speed_ki_;

        current_limit_a_ = motor_params.max_current_a;
        accel_current_limit_a_ = control_params.max_accel_torque_a;

        /* Power control term calculation */
        ir_ = motor_params.rotor_inertia_kg_m2;
        kq0_ = 0.75f * (float)motor_params.num_poles *
               motor_params.phi_v_s_per_rad;
    }

    float __attribute__((optimize("O3")))
    update(const struct motor_state_t& state) {
        float error, power, accel_torque_a, result_a, out_a, kp, ki;

        if (control_power_) {
            /* Eqn 17 from the paper */
            power =
                (kq0_ * state.i_dq_a[1] * state.angular_velocity_rad_per_s) /* -
                (ir_ * state.angular_velocity_rad_per_s *
                    state.angular_acceleration_rad_per_s2) */;
            error = power_setpoint_w_ - power;
            kp = power_kp_;
            ki = power_ki_;
        } else {
            error = speed_setpoint_rad_per_s_ -
                    state.angular_velocity_rad_per_s;
            kp = speed_kp_;
            ki = speed_ki_;
        }

        /*
        Determine acceleration torque, and limit to the configured maximum
        */
        accel_torque_a = kp * error;

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
        Limit integral term accumulation when the output current is saturated.
        */
        integral_error_a_ += ki * (result_a - integral_error_a_);

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
