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
#include "park.h"
#include "esc_assert.h"


class StateEstimator {
    struct motor_state_t state_estimate_;

    /* Calculated from start-up parameters */
    float open_loop_target_rad_per_s_;
    float open_loop_speed_rad_per_s_;
    float open_loop_accel_rad_per_s2_;

    /* Intermediate values */
    float a_; /* 1.0 - R / L * T */
    float b_; /* phi / L * T */
    float c_; /* T / L */
    float t_;

    /* Current and speed lowpass filter parameters */
    float i_dq_lpf_coeff_;
    float angular_velocity_lpf_coeff_;

    /* Column-major */
    float state_covariance_[4];

    float last_i_ab_a_[2];
    float next_sin_theta_;
    float next_cos_theta_;

    /* Low-passed angular velocity estimate */
    float filtered_velocity_rad_per_s_;

public:
    StateEstimator():
        state_estimate_ {0.0f, 0.0f, 0.0f, 0.0f},
        open_loop_target_rad_per_s_(0.0f),
        open_loop_speed_rad_per_s_(0.0f),
        open_loop_accel_rad_per_s2_(0.0f),
        a_(0.0f),
        b_(0.0f),
        c_(0.0f),
        t_(0.0f),
        i_dq_lpf_coeff_(0.0f),
        angular_velocity_lpf_coeff_(0.0f),
        state_covariance_ {1.0f, 0.0f, 0.0f, 1.0f},
        last_i_ab_a_ {0.0f, 0.0f},
        next_sin_theta_(0.0f),
        next_cos_theta_(1.0f),
        filtered_velocity_rad_per_s_(0.0f)
    {}

    void reset_state() {
        state_estimate_.angular_velocity_rad_per_s =
            state_estimate_.angle_rad = 0.0f;
        next_sin_theta_ = 0;
        next_cos_theta_ = 1.0f;
        last_i_ab_a_[0] = last_i_ab_a_[1] = 0.0f;
        state_covariance_[0] = state_covariance_[2] = 1.0f;
        state_covariance_[1] = state_covariance_[3] = 0.0f;
        filtered_velocity_rad_per_s_ = 0.0f;
        open_loop_speed_rad_per_s_ = 0.0f * open_loop_target_rad_per_s_;
    }

    void update_state_estimate(
        const float i_ab_a[2],
        const float v_ab_v[2],
        float velocity_setpoint
    );

    void get_state_estimate(struct motor_state_t& out_estimate) const {
        out_estimate = state_estimate_;
        out_estimate.angular_velocity_rad_per_s = filtered_velocity_rad_per_s_;
    }

    void get_est_v_alpha_beta_from_v_dq(
        float out_v_alpha_beta[2],
        const float in_v_dq[2]
    ) const {
        inverse_park_transform(out_v_alpha_beta, in_v_dq, next_sin_theta_,
                               next_cos_theta_);
    }

    void set_params(
        const struct motor_params_t& params,
        const struct control_params_t& control_params,
        float t_s
    ) {
        a_ = 1.0f - params.rs_r / params.ls_h * t_s;
        b_ = params.phi_v_s_per_rad / params.ls_h * t_s;
        c_ = t_s / params.ls_h;
        t_ = t_s;

        /*
        Control parameters -- LPF corner frequency is one decade higher than
        the controller bandwidth; current control bandwidth is one decade
        higher than speed control bandwidth.
        */
        i_dq_lpf_coeff_ =
            1.0f - fast_expf(-2.0f * (float)M_PI * control_params.bandwidth_hz * t_s * 150.0f);
        angular_velocity_lpf_coeff_ =
            1.0f - fast_expf(-2.0f * (float)M_PI * control_params.bandwidth_hz * t_s * 10.0f);

        /* Start-up parameters */
        open_loop_accel_rad_per_s2_ = 0.1f * params.startup_speed_rad_per_s *
                                      control_params.bandwidth_hz;
        open_loop_target_rad_per_s_ = params.startup_speed_rad_per_s;
    }
};


class ParameterEstimator {
    float sample_voltages_[4];
    float sample_currents_[4];

    float open_loop_angular_velocity_rad_per_s_;
    float open_loop_angle_rad_;

    float v_;
    float t_;
    uint16_t test_idx_;
    uint16_t open_loop_test_samples_;

public:
    ParameterEstimator():
        sample_voltages_ {0, 0, 0, 0},
        sample_currents_ {0, 0, 0, 0},
        open_loop_angular_velocity_rad_per_s_(0.0f),
        open_loop_angle_rad_(0.0f),
        v_(0.0f),
        t_(0.0f),
        test_idx_(0),
        open_loop_test_samples_(0)
    {}

    void start_estimation(float t);

    void update_parameter_estimate(
        const float i_ab_a[2],
        const float v_ab_v[2]
    );

    void get_v_alpha_beta_v(float v_ab_v[2]);

    bool is_estimation_complete(void) const {
        return test_idx_ == 4;
    }

    void get_samples(float out_v[4], float out_i[4]) {
        memcpy(out_v, sample_voltages_, sizeof(sample_voltages_));
        memcpy(out_i, sample_currents_, sizeof(sample_currents_));
    }

    static void calculate_r_l_from_samples(
        float& r_r,
        float& l_h,
        const float v[4],
        const float i[4]
    );
};
