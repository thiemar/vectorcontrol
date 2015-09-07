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


#include <cstring>
#include "fixed.h"
#include "park.h"
#include "esc_assert.h"


class StateEstimator {
    struct motor_state_t state_estimate_;
    float phi_estimate_v_s_per_rad_;

    /* Intermediate values */
    float a_; /* 1.0 - R / L * T */
    float b_; /* 1.0 / L * T; needs to be multiplied by phi */
    float c_; /* T / L */
    float t_;
    float t_inv_;
    float rs_r_;

    /* Current and speed lowpass filter parameters */
    float i_dq_lpf_coeff_;
    float angular_velocity_lpf_coeff_;

    /* Column-major */
    float state_covariance_[4];

    /* Intermediate state */
    float last_i_ab_a_[2];
    float next_sin_theta_;
    float next_cos_theta_;

public:
    StateEstimator():
        phi_estimate_v_s_per_rad_(0.0f),
        a_(0.0f),
        b_(0.0f),
        c_(0.0f),
        t_(0.0f),
        t_inv_(0.0f),
        rs_r_(0.0f),
        i_dq_lpf_coeff_(0.0f),
        angular_velocity_lpf_coeff_(0.0f),
        next_sin_theta_(0.0f),
        next_cos_theta_(1.0f)
    {
        reset_state();
    }

    void reset_state() volatile {
        state_estimate_.angular_acceleration_rad_per_s2 = 0.0f;
        state_estimate_.angular_velocity_rad_per_s = 0.0f;
        state_estimate_.angle_rad = 0.0f;
        state_estimate_.i_dq_a[0] = state_estimate_.i_dq_a[1] = 0.0f;
        next_sin_theta_ = 0;
        next_cos_theta_ = 1.0f;
        last_i_ab_a_[0] = last_i_ab_a_[1] = 0.0f;
        state_covariance_[0] = state_covariance_[2] = 10.0f;
        state_covariance_[1] = state_covariance_[3] = 1.0f;
    }

    void update_state_estimate(
        const float i_ab_a[2],
        const float v_ab_v[2],
        float speed_setpoint,
        float closed_loop_frac
    );

    void get_state_estimate(struct motor_state_t& out_estimate) const {
        out_estimate = state_estimate_;
    }

    void get_est_v_alpha_beta_from_v_dq(
        float out_v_alpha_beta[2],
        const float in_v_dq[2]
    ) const {
        inverse_park_transform(out_v_alpha_beta, in_v_dq, next_sin_theta_,
                               next_cos_theta_);
    }

    float get_phi_estimate(void) const {
        return phi_estimate_v_s_per_rad_;
    }

    void set_motor_params(
        float rs_r,
        float ls_h,
        float phi_v_s_per_rad,
        float t_s
    ) volatile {
        a_ = 1.0f - rs_r / ls_h * t_s;
        b_ = 1.0f / ls_h * t_s;
        c_ = t_s / ls_h;
        t_ = t_s;
        t_inv_ = 1.0f / t_s;
        rs_r_ = rs_r;
        phi_estimate_v_s_per_rad_ = phi_v_s_per_rad;
    }

    void set_control_params(
        float control_bandwidth_hz,
        float t_s
    ) volatile {
        float rc;

        /*
        Control parameters -- LPF bandwidth is one octave higher than speed
        control bandwidth, and angular velocity estimation bandwidth is one
        octave higher.
        */
        rc = 1.0f / (float(2.0 * M_PI) * control_bandwidth_hz);
        i_dq_lpf_coeff_ = t_s / (t_s + 0.1f * rc);
        angular_velocity_lpf_coeff_ = t_s / (t_s + 0.5f * rc);
    }
};


class ParameterEstimator {
    float sample_voltages_[4];
    float sample_currents_[4];

    float open_loop_angular_velocity_rad_per_u_;
    float open_loop_angle_rad_;

    float v_;
    uint16_t test_idx_;
    uint16_t open_loop_test_samples_;

public:
    ParameterEstimator():
        open_loop_angular_velocity_rad_per_u_(0.0f),
        open_loop_angle_rad_(0.0f),
        v_(0.0f),
        test_idx_(0),
        open_loop_test_samples_(0)
    {
        sample_voltages_[0] = sample_voltages_[1] = sample_voltages_[2] =
            sample_voltages_[3] = 0.0f;
        sample_currents_[0] = sample_currents_[1] = sample_currents_[2] =
            sample_currents_[3] = 0.0f;
    }

    void start_estimation(float t);

    void update_parameter_estimate(
        const float i_ab_a[2],
        const float v_ab_v[2]
    );

    void get_v_alpha_beta_v(float v_ab_v[2]);

    bool is_estimation_complete(void) const {
        return test_idx_ == 4;
    }

    void calculate_r_l(float& r_r, float& l_h);
};
