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
    Implements standard PI speed control, as well as a thrust estimator and
    controller.
    */
protected:
    bool control_thrust_;

    float integral_error_a_;
    float speed_setpoint_rad_per_s_;

    float thrust_setpoint_n_;
    float thrust_setpoint_dt_n_per_s_;

    float kp_;
    float ki_;

    float t_inv_; /* 1.0 / t_s */
    float ir_;  /* Rotor inertia in kg * m^2*/
    float kq_v_s_per_mech_rad_; /* Torque constant */
    float ka_; /* 0.5 * RHO * c * B * r */
    float reff_m_; /* Propeller effective radius in m (0.75 * r) */
    float num_pole_pairs_; /* pole count / 2 */
    float inv_num_pole_pairs_; /* 1.0 / (pole count / 2) */

    float cl_kx, cl_k;
    float cd_kx, cd_k;

    float current_limit_a_;
    float accel_current_limit_a_;

    float thrust_lpf_coeff_;

    /* Temporary state */
    float estimated_thrust_n_;
    float estimated_inflow_angle_deg_;


public:
    SpeedController():
        control_thrust_(false),
        integral_error_a_(0.0f),
        speed_setpoint_rad_per_s_(0.0f),
        thrust_setpoint_n_(0.0f),
        thrust_setpoint_dt_n_per_s_(0.0f),
        kp_(0.0f),
        ki_(0.0f),
        t_inv_(0.0f),
        ir_(0.0f),
        kq_v_s_per_mech_rad_(0.0f),
        ka_(0.0f),
        reff_m_(1.0f),
        num_pole_pairs_(4.0f),
        inv_num_pole_pairs_(0.25f),
        cl_kx(0.0f),
        cl_k(0.0f),
        cd_kx(0.0f),
        cd_k(0.0f),
        current_limit_a_(0.0f),
        accel_current_limit_a_(0.0f),
        thrust_lpf_coeff_(0.0f),
        estimated_thrust_n_(0.0f),
        estimated_inflow_angle_deg_(0.0f)
    {}

    void reset_state() {
        integral_error_a_ = 0.0f;
        speed_setpoint_rad_per_s_ = 0.0f;
        thrust_setpoint_n_ = 0.0f;
        thrust_setpoint_dt_n_per_s_ = 0.0f;
        estimated_thrust_n_ = 0.0f;
        estimated_inflow_angle_deg_= 0.0f;
    }

    void __attribute__((always_inline)) set_speed_setpoint(float setpoint) {
        speed_setpoint_rad_per_s_ = setpoint;
        control_thrust_ = false;
    }

    void __attribute__((always_inline)) set_thrust_setpoint(float setpoint) {
        float delta;

        delta = (setpoint - thrust_setpoint_n_) * thrust_lpf_coeff_;
        thrust_setpoint_n_ += delta;
        thrust_setpoint_dt_n_per_s_ = delta * t_inv_;

        control_thrust_ = true;
    }

    void __attribute__((always_inline)) set_phi_v_s_per_rad(float phi) {
        kq_v_s_per_mech_rad_ = phi * num_pole_pairs_;
    }

    float __attribute__((always_inline)) get_estimated_thrust_n(void) const {
        return estimated_thrust_n_;
    }

    float __attribute__((always_inline))
    get_estimated_inflow_angle_deg(void) const {
        return estimated_inflow_angle_deg_;
    }

    float __attribute__((always_inline)) get_max_thrust_n(void) const {
        float torque_nm;
        torque_nm = current_limit_a_ * kq_v_s_per_mech_rad_;
        /*
        Return maximum static thrust by taking Cl, Cd at inflow angle = 0.

        This will not be accurate for props with higher pitch, but should be
        close enough generally.

        FIXME: check voltage limit and Kv, then use the speed-based limit if
        it's lower than this one.
        */
        return torque_nm / (cd_k * reff_m_) * cl_k;
    }

    void  set_params(
        const struct motor_params_t& motor_params,
        const struct control_params_t& control_params,
        float t_s
    ) volatile {
        float max_accel_torque, rc;

        max_accel_torque = motor_params.accel_voltage_v / motor_params.rs_r;

        kp_ = float(1.0/512.0) * max_accel_torque * control_params.accel_gain;
        ki_ = t_s / control_params.accel_time_s;

        current_limit_a_ = motor_params.max_current_a;
        accel_current_limit_a_ = std::min(motor_params.max_current_a,
                                          max_accel_torque);

        /*
        Thrust control term calculation.

        Assuming perfect efficiency, maximum electrical power is given by
        vq * iq, and mechanical power is given by t * w.

        Relating vq * iq = t * w and solving for t, we get t = vq * iq / w.

        Phi is vq / w, so kq0 = phi.

        Scale rotor inertia by (2/poles) to convert the acceleration torque
        term from electrical to mechanical.
        */
        num_pole_pairs_ = float(motor_params.num_poles / 2);
        inv_num_pole_pairs_ = 1.0f / num_pole_pairs_;
        kq_v_s_per_mech_rad_ = motor_params.phi_v_s_per_rad * num_pole_pairs_;
        reff_m_ = control_params.prop_radius_m * 0.75f;
        ka_ = float(RHO_KG_PER_M3 / 2.0) * control_params.prop_chord_m *
              reff_m_ * float(control_params.prop_num_blades);
        ir_ = control_params.prop_mass_kg * (2.0f * reff_m_) *
              (2.0f * reff_m_) / 12.0f;

        rc = 1.0f / (float(2.0 * M_PI) * control_params.bandwidth_hz);
        thrust_lpf_coeff_ = t_s / (t_s + rc);
        t_inv_ = 1.0f / t_s;

        /* Cd and Cl approximation coefficients */
        cl_kx = control_params.prop_cl_kx;
        cl_k = control_params.prop_cl_k;
        cd_kx = control_params.prop_cd_kx;
        cd_k = control_params.prop_cd_k;
    }

    void __attribute__((always_inline))
    set_integral_current_a(float current_a) {
        integral_error_a_ = current_a;
    }

    float __attribute__((always_inline))
    update(const struct motor_state_t& state, float closed_loop_frac) {
        float error, accel_torque_a, out_a, v_m_per_s, cd, cl, torque_a,
              torque_n_m, thrust_n, torque_setpoint_n_m,
              delta_torque_n_m_per_s;

        torque_a = state.i_dq_a[1];
        torque_n_m = torque_a * kq_v_s_per_mech_rad_ -
                     state.angular_acceleration_rad_per_s2 * ir_ *
                     inv_num_pole_pairs_;
        v_m_per_s = state.angular_velocity_rad_per_s * reff_m_ *
                    inv_num_pole_pairs_;

        cd = std::max(torque_n_m / (ka_ * reff_m_ * v_m_per_s * v_m_per_s),
                      0.0125f);
        estimated_inflow_angle_deg_ = std::max(0.0f, (cd - cd_k) / cd_kx);

        cd = std::max(0.0125f, cd_k + cd_kx * estimated_inflow_angle_deg_);
        cl = std::max(0.0125f, cl_k + cl_kx * estimated_inflow_angle_deg_);

        thrust_n = cl * torque_n_m / (cd * reff_m_);

        if (closed_loop_frac >= 1.0f) {
            estimated_thrust_n_ += (thrust_n - estimated_thrust_n_) *
                                   thrust_lpf_coeff_;
        }

        if (control_thrust_) {
            /*
            Convert thrust error to a current error based on the torque
            required to achieve that thrust in static conditions.
            */
            torque_setpoint_n_m = thrust_setpoint_n_ * cd * reff_m_ / cl;
            delta_torque_n_m_per_s = thrust_setpoint_dt_n_per_s_ *
                                     cd * reff_m_ / cl;

            /*
            Add a derivative term based on setpoint change and prop inertia
            */
            error = torque_setpoint_n_m - torque_n_m +
                    (delta_torque_n_m_per_s / torque_setpoint_n_m) *
                    state.angular_velocity_rad_per_s * ir_ *
                    inv_num_pole_pairs_;
            accel_torque_a = error / kq_v_s_per_mech_rad_;
        } else {
            /*
            Find speed error and convert it to a current error based on Kp
            */
            error = speed_setpoint_rad_per_s_ -
                    state.angular_velocity_rad_per_s;

            accel_torque_a = kp_ * error;
        }

        /* Limit acceleration torque to the configured maximum */
        if (accel_torque_a > accel_current_limit_a_) {
            accel_torque_a = accel_current_limit_a_;
        }
        if (accel_torque_a < -accel_current_limit_a_) {
            accel_torque_a = -accel_current_limit_a_;
        }

        out_a = accel_torque_a + integral_error_a_;

        /* Limit output to maximum current */
        if (out_a > current_limit_a_) {
            out_a = current_limit_a_;
        }
        if (out_a < -current_limit_a_) {
            out_a = -current_limit_a_;
        }

        /*
        Limit integral term accumulation when the output current is saturated.
        */
        integral_error_a_ += ki_ * (out_a - integral_error_a_);

        return out_a;
    }
};
