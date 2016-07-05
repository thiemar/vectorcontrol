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
    float ls_h_;
    float t_;
    float t_inv_;
    float rs_r_;

    /* Current and velocity lowpass filter parameters */
    float i_dq_lpf_coeff_;
    float angular_velocity_lpf_coeff_;

    /* Intermediate state */
    float next_sin_theta_;
    float next_cos_theta_;

public:
    StateEstimator():
        phi_estimate_v_s_per_rad_(0.0f),
        ls_h_(0.0f),
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
        state_estimate_.angular_velocity_rad_per_s = 0.0f;
        state_estimate_.angle_rad = 0.0f;
        state_estimate_.i_dq_a[0] = state_estimate_.i_dq_a[1] = 0.0f;
        state_estimate_.v_dq_v[0] = state_estimate_.v_dq_v[1] = 0.0f;
        next_sin_theta_ = 0;
        next_cos_theta_ = 1.0f;
    }

    void update_state_estimate(
        const float i_ab_a[2],
        const float v_ab_v[2],
        float speed_setpoint
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
        ls_h_ = ls_h;
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
        Control parameters -- LPF bandwidth is one decade higher than speed
        control bandwidth, and angular velocity estimation bandwidth is one
        octave higher.
        */
        rc = 1.0f / (float(2.0 * M_PI) * control_bandwidth_hz);
        i_dq_lpf_coeff_ = t_s / (t_s + 0.1f * rc);
        angular_velocity_lpf_coeff_ = t_s / (t_s + 0.5f * rc);
    }
};


class ParameterEstimator {
    float sample_v_sq_[5];
    float sample_i_sq_[5];

    float open_loop_angular_velocity_rad_per_u_;
    float open_loop_angle_rad_;

    float v_;
    uint8_t test_idx_;
    uint8_t retry_idx_;
    uint16_t open_loop_test_samples_;

public:
    ParameterEstimator():
        open_loop_angular_velocity_rad_per_u_(0.0f),
        open_loop_angle_rad_(0.0f),
        v_(0.0f),
        test_idx_(0),
        retry_idx_(0),
        open_loop_test_samples_(0)
    { }

    void start_estimation(float t);

    void update_parameter_estimate(
        const float i_ab_a[2],
        const float v_ab_v[2]
    );

    void get_v_alpha_beta_v(float v_ab_v[2]);

    bool is_estimation_complete(void) const {
        return test_idx_ >= 5;
    }

    void calculate_r_l(float& r_r, float& l_h);
};
