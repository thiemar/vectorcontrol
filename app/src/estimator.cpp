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
#include <cassert>
#include <cstring>
#include <limits>
#include <algorithm>
#include "hal.h"
#include "estimator.h"
#include "perf.h"
#include "park.h"


void StateEstimator::update_state_estimate(
    const float i_ab_a[2],
    const float v_ab_v[2],
    float speed_setpoint
) {
    /*
    Position/velocity observer based on:

    Wang, T.; Dong, S.; Yong, T.; Wang, Y.; Chen, Z. (2015)
    "A High Dynamic Performance PMSM Sensorless Algorithm Based on Rotor
    Position Tracking Observer"

    https://www.atlantis-press.com/php/download_paper.php?id=25837868
    */
    float sin_theta, cos_theta, i_dq_a[2], v_dq_v[2], angle_error, next_angle,
          phi, w_e, angle_delta;
    bool fast_enough;

    /* Update the Idq estimate based on the theta estimate for time t */
    sin_cos(sin_theta, cos_theta, state_estimate_.angle_rad);
    park_transform(i_dq_a, i_ab_a, sin_theta, cos_theta);
    park_transform(v_dq_v, v_ab_v, sin_theta, cos_theta);

    /* Update the external Idq and Vdq estimates */
    state_estimate_.i_dq_a[0] +=
        (i_dq_a[0] - state_estimate_.i_dq_a[0]) * i_dq_lpf_coeff_;
    state_estimate_.i_dq_a[1] +=
        (i_dq_a[1] - state_estimate_.i_dq_a[1]) * i_dq_lpf_coeff_;
    state_estimate_.v_dq_v[0] +=
        (v_dq_v[0] - state_estimate_.v_dq_v[0]) * i_dq_lpf_coeff_;
    state_estimate_.v_dq_v[1] +=
        (v_dq_v[1] - state_estimate_.v_dq_v[1]) * i_dq_lpf_coeff_;

    /* Calculate the angle error using eqn (5) and (6) in the paper */
    w_e = state_estimate_.angular_velocity_rad_per_s;
    angle_error = -(state_estimate_.v_dq_v[0] -
                    rs_r_ * state_estimate_.i_dq_a[0] +
                    w_e * ls_h_ * state_estimate_.i_dq_a[1]);

    w_e = 1.0f / std::max(2.0f, std::abs(w_e * phi_estimate_v_s_per_rad_));

    /*
    This essentially implements a PI observer where angular velocity is the
    angle error accumulator, and angle_error is the value to be minimized.
    */
    angle_delta = state_estimate_.angular_velocity_rad_per_s * t_;
    state_estimate_.angle_rad += angle_delta;

    angle_error = (angle_error * w_e - angle_delta) * 0.1f;

    /*
    Prevent the angular velocity from growing in the opposite direction to the
    speed setpoint, to avoid starting in reverse (well, at low-ish currents
    anyway).
    */
    if (angle_error * state_estimate_.angular_velocity_rad_per_s < 0.0f ||
            angle_error * speed_setpoint >= 0.0f) {
        state_estimate_.angular_velocity_rad_per_s +=
            angle_error * t_inv_ * angular_velocity_lpf_coeff_ * 0.1f;
    }

    /*
    If angular velocity is small, we may have stalled, and the angle error
    estimate won't be valid anyway. Instead of updating the angle using the
    angle error, we update it using the speed setpoint to ensure we're trying
    to move in the right direction.
    */
    fast_enough = std::abs(state_estimate_.angular_velocity_rad_per_s) >
                  float(2.0 * M_PI * 2.0);
    if (fast_enough) {
        state_estimate_.angle_rad += angle_error;
    } else {
        state_estimate_.angle_rad += speed_setpoint * t_;
    }

    /* Wrap angle to +/- pi -- use conditional instructions (hopefully) */
    if (state_estimate_.angle_rad > float(M_PI)) {
        state_estimate_.angle_rad -= float(2.0 * M_PI);
    }
    if (state_estimate_.angle_rad < -float(M_PI)) {
        state_estimate_.angle_rad += float(2.0 * M_PI);
    }

    next_angle = state_estimate_.angle_rad + angle_delta;

    /* Wrap next angle */
    if (next_angle > float(M_PI)) {
        next_angle -= float(2.0 * M_PI);
    }
    if (next_angle < -float(M_PI)) {
        next_angle += float(2.0 * M_PI);
    }

    sin_cos(next_sin_theta_, next_cos_theta_, next_angle);

    /* Estimate the value of phi (the back-EMF constant) */
    phi = phi_estimate_v_s_per_rad_;
    if (fast_enough && speed_setpoint != 0.0f) {
        phi = std::abs(
            (state_estimate_.v_dq_v[1] - state_estimate_.i_dq_a[1] * rs_r_) /
            state_estimate_.angular_velocity_rad_per_s
        );
    }
    phi_estimate_v_s_per_rad_ += (phi - phi_estimate_v_s_per_rad_) *
        angular_velocity_lpf_coeff_ * 0.1f;
}


/*
Number of samples is selected such that the test period is almost exactly
8 full cycles of the lowest frequency (PE_START_FREQ / 8).
*/
#define PE_TEST_SAMPLES uint32_t(PE_TEST_CYCLES * (1.0f / hal_control_t_s) / \
                                                  (PE_START_FREQ_HZ / 8.0f))
#define PE_MAX_RETRIES 6u


void ParameterEstimator::start_estimation(float t) {
    open_loop_angle_rad_ = 0.0f;
    open_loop_test_samples_ = 0;
    open_loop_angular_velocity_rad_per_u_ =
        float(2.0 * M_PI) * PE_START_FREQ_HZ * t;
    v_ = PE_START_V_V;
    test_idx_ = 0;
}


void ParameterEstimator::update_parameter_estimate(
    const float i_ab_a[2],
    const float v_ab_v[2]
) {
    /*
    Measure R and L by passing a high-frequency test signal through all three
    phases. This is the same signal as used to drive the motor, but at a
    sufficiently high frequency and low voltage that the motor doesn't rotate.

    Run four tests, each at half the frequency of the previous one.

    If the current during a test is less than the minimum, double the voltage
    and repeat (as many times as necessary). If the current is greater than
    the maximum, halve the voltage and repeat.

    At the end of the process, determine R and L by linear regression of the
    four impedance readings against the test frequencies.
    */
    if (test_idx_ >= 5) {
        /* Early exit if the test is complete */
        return;
    } else if (test_idx_ < 4) {
        /*
        Increment the phase angle, and the cycle count if the angle has
        wrapped round to zero.
        */
        open_loop_angle_rad_ += open_loop_angular_velocity_rad_per_u_;
        if (open_loop_angle_rad_ > float(M_PI)) {
            open_loop_angle_rad_ -= float(2.0 * M_PI);
        }
    }

    if (open_loop_test_samples_ == PE_TEST_SAMPLES) {
        /*
        Divide the Z accumulator by the number of recorded samples (half
        of the actual number of samples) to get the mean square value.
        */
        sample_v_sq_[test_idx_] *= (1.0f / float(PE_TEST_SAMPLES / 2));
        sample_i_sq_[test_idx_] *= (1.0f / float(PE_TEST_SAMPLES / 2));

        /*
        At the end of a test run, check the RMS current and increase or
        decrease voltage as necessary.
        */
        if (sample_i_sq_[test_idx_] > PE_MAX_I_A * PE_MAX_I_A &&
                v_ > PE_MIN_V_V && retry_idx_ < PE_MAX_RETRIES) {
            v_ *= 0.5f;
            retry_idx_++;
        } else if (sample_i_sq_[test_idx_] < PE_MIN_I_A * PE_MIN_I_A &&
                   v_ < PE_MAX_V_V && retry_idx_ < PE_MAX_RETRIES) {
            v_ *= 2.0f;
            retry_idx_++;
        } else {
            /* Current was OK -- halve test frequency for the next test */
            open_loop_angular_velocity_rad_per_u_ *= 0.5f;

            /* Increment the test number */
            test_idx_++;

            /* Reset the retry count */
            retry_idx_ = 0;
        }

        open_loop_test_samples_ = 0;
    } else if (open_loop_test_samples_ >= PE_TEST_SAMPLES / 2) {
        /* Accumulate the squared current and voltage readings */
        sample_v_sq_[test_idx_] += v_ab_v[0] * v_ab_v[0] +
                                   v_ab_v[1] * v_ab_v[1];
        sample_i_sq_[test_idx_] += i_ab_a[0] * i_ab_a[0] +
                                   i_ab_a[1] * i_ab_a[1];
    } else {
        sample_v_sq_[test_idx_] = 0.0f;
        sample_i_sq_[test_idx_] = 0.0f;
    }

    open_loop_test_samples_++;
}


void ParameterEstimator::get_v_alpha_beta_v(float v_ab_v[2]) {
    /*
    Get alpha and beta voltage components for the latest phase angle. Only
    output a voltage for the first 12 out of 16 test cycles to give time for
    the current to decay to zero.
    */
    float v_dq[2] = {v_, 0.0f};
    inverse_park_transform(v_ab_v, v_dq, open_loop_angle_rad_);
}


void ParameterEstimator::calculate_r_l(float& r_r, float& l_h) {
    size_t idx;
    float w_sq, sum_xy, sum_x, sum_y, sum_x_sq, a, b, z_sq;

    sum_xy = sum_x = sum_y = sum_x_sq = 0;
    w_sq = float(2.0 * M_PI) * PE_START_FREQ_HZ;
    w_sq *= w_sq;

    /*
    Run simple linear regression, and calculate a and b such that
    y = bx + a

    where y = z^2 and x = w^2.

    L is then sqrt(b) and R is sqrt(a).
    */
    for (idx = 0; idx < 4u; idx++) {
        sum_x += w_sq;
        sum_x_sq += w_sq * w_sq;

        z_sq = sample_v_sq_[idx] / sample_i_sq_[idx];
        sum_xy += w_sq * z_sq;
        sum_y += z_sq;

        w_sq *= 0.25f; /* Halve angular velocity for the next test */
    }

    b = (4.0f * sum_xy - sum_x * sum_y) / (4.0f * sum_x_sq - sum_x * sum_x);
    a = 0.25f * (sum_y - b * sum_x);

    a = sample_v_sq_[4] / sample_i_sq_[4];
    if (std::isnan(a) || a < 1e-6f) {
        a = 1e-6f;
    }
    r_r = __VSQRTF(a);

    if (std::isnan(b) || b < 1e-12f) {
        b = 1e-12f;
    }
    l_h = __VSQRTF(b);
}
