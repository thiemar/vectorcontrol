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


#define STATE_DIM 2
#define MEASUREMENT_DIM 2


const float g_process_noise[2] = { 20.0f, 1e-6f };
//const float g_measurement_noise[2] = { 0.02f, 0.02f };
const float g_measurement_noise[2] = { 0.05f, 0.05f };


void StateEstimator::update_state_estimate(
    const float i_ab_a[2],
    const float v_ab_v[2],
    float speed_setpoint,
    float closed_loop_frac
) {
    /*
    EKF observer largely derived from:

    Smidl, V. and Peroutka, Z. (2011)
    "Reduced-Order Square-Root EKF for Sensorless Control of PMSM Drives"

    http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6119446
    */
    float covariance_temp[STATE_DIM * STATE_DIM],
          hessian[STATE_DIM * STATE_DIM],
          prediction[MEASUREMENT_DIM], innovation[STATE_DIM],
          measurement_covariance[MEASUREMENT_DIM * MEASUREMENT_DIM],
          measurement_covariance_temp[MEASUREMENT_DIM * MEASUREMENT_DIM],
          kalman_gain[STATE_DIM * MEASUREMENT_DIM],
          kalman_gain_temp[STATE_DIM * MEASUREMENT_DIM],
          update[STATE_DIM * STATE_DIM], determinant, sin_theta, cos_theta,
          b_est_sin_theta, b_est_cos_theta, m_a, m_b, m_d, acceleration,
          i_dq_a[2], last_angle, next_angle, b, phi, vs, is, closed_loop_mul;

    /* Update the Idq estimate based on the theta estimate for time t */
    sin_cos(sin_theta, cos_theta, state_estimate_.angle_rad);
    park_transform(i_dq_a, i_ab_a, sin_theta, cos_theta);

    state_estimate_.i_dq_a[0] += (i_dq_a[0] - state_estimate_.i_dq_a[0]) *
                                 i_dq_lpf_coeff_;
    state_estimate_.i_dq_a[1] += (i_dq_a[1] - state_estimate_.i_dq_a[1]) *
                                 i_dq_lpf_coeff_;

#define Pm(i,j) covariance_temp[i * STATE_DIM + j]
#define P(i,j) state_covariance_[i * STATE_DIM + j]
#define M(i,j) measurement_covariance[i * MEASUREMENT_DIM + j]
#define Mt(i,j) measurement_covariance_temp[i * MEASUREMENT_DIM + j]
#define K(i,j) kalman_gain[i * STATE_DIM + j]
#define Kt(i,j) kalman_gain_temp[i * STATE_DIM + j]
#define H(i,j) hessian[i * STATE_DIM + j]
#define U(i,j) update[i * STATE_DIM + j]

    /*
    Update the covariance matrix from the last iteration [P(+)] to find the a
    priori covariance for this iteration [P(-)]:

    P(-) = A x P(+) x A^T + Q

    The A matrix is
    [ 1    0
      TS   1 ]

    The Q matrix is the process noise covariance -- diagonal only in this
    application.

    Since P is symmetric, we only compute/store the lower triangle. The other
    value is set to INT32_MIN to ensure it blows something up if ever used.
    */
    Pm(0,0) = P(0,0) + g_process_noise[0];
    Pm(0,1) = t_ * P(0,0) + P(0,1);
    // Pm(1,0) = std::numeric_limits<float>::signaling_NaN();
    Pm(1,1) = P(0,0) * t_ * t_ +
              2.0f * t_ * P(0,1) +
              P(1,1) + g_process_noise[1];

    /* These values are used a few times */
    b = b_ * phi_estimate_v_s_per_rad_;
    b_est_sin_theta = b * sin_theta;
    b_est_cos_theta = b * cos_theta;

    /* Set up the Hessian (H) */
    H(0,0) = b_est_sin_theta;
    H(0,1) = -b_est_cos_theta;
    H(1,0) = b_est_cos_theta * state_estimate_.angular_velocity_rad_per_s;
    H(1,1) = b_est_sin_theta * state_estimate_.angular_velocity_rad_per_s;

    /*
    Find the inverse measurement covariance:

    (H x P(-) x H^T + R)^-1

    H and P(-) are from the earlier steps; R is the measurement noise
    covariance, which is also diagonal.

    First, find Mt = H x P(-). The multiplication below ignores the top-right
    corner of P since it's symmetric.
    */
    Mt(0,0) = H(0,0) * Pm(0,0) + H(1,0) * Pm(0,1);
    Mt(0,1) = H(0,1) * Pm(0,0) + H(1,1) * Pm(0,1);
    Mt(1,0) = H(0,0) * Pm(0,1) + H(1,0) * Pm(1,1);
    Mt(1,1) = H(0,1) * Pm(0,1) + H(1,1) * Pm(1,1);

    /*
    Now find M = Mt x H^T. M is symmetric so don't calculate the top right.
    */
    M(0,0) = Mt(0,0) * H(0,0) + Mt(1,0) * H(1,0) + g_measurement_noise[0];
    M(0,1) = Mt(0,1) * H(0,0) + Mt(1,1) * H(1,0);
    // M(1,0) = std::numeric_limits<float>::signaling_NaN();
    M(1,1) = Mt(0,1) * H(0,1) + Mt(1,1) * H(1,1) + g_measurement_noise[1];

    /* Calculate the determinant -- symmetric so square the bottom-left */
    determinant = 1.0f / std::max(1e-6f, M(0,0) * M(1,1) - M(0,1) * M(0,1));
    m_a = M(1,1) * determinant;
    m_b = -M(0,1) * determinant;
    m_d = M(0,0) * determinant;

    /*
    Calculate Kalman gain:

    K = P(-) x H^T x M^-1
    */
    Kt(0,0) = Pm(0,0) * H(0,0) + Pm(0,1) * H(1,0);
    Kt(0,1) = Pm(0,1) * H(0,0) + Pm(1,1) * H(1,0);
    Kt(1,0) = Pm(0,0) * H(0,1) + Pm(0,1) * H(1,1);
    Kt(1,1) = Pm(0,1) * H(0,1) + Pm(1,1) * H(1,1);

    K(0,0) = Kt(0,0) * m_a + Kt(1,0) * m_b;
    K(0,1) = Kt(0,1) * m_a + Kt(1,1) * m_b;
    K(1,0) = Kt(0,0) * m_b + Kt(1,0) * m_d;
    K(1,1) = Kt(0,1) * m_b + Kt(1,1) * m_d;

    /*
    Calculate and apply the covariance update:

    U = I - K x H

    then

    P(+) = U x P(-)
    */
    U(0,0) = 1.0f - (K(0,0) * H(0,0) + K(1,0) * H(0,1));
    U(0,1) = -(K(0,1) * H(0,0) + K(1,1) * H(0,1));
    U(1,0) = -(K(0,0) * H(1,0) + K(1,0) * H(1,1));
    U(1,1) = 1.0f - (K(0,1) * H(1,0) + K(1,1) * H(1,1));

    P(0,0) = U(0,0) * Pm(0,0) + U(1,0) * Pm(0,1);
    P(0,1) = U(0,1) * Pm(0,0) + U(1,1) * Pm(0,1);
    // P(1,0) = std::numeric_limits<float>::signaling_NaN();
    P(1,1) = U(0,1) * Pm(0,1) + U(1,1) * Pm(1,1);

    P(0,0) = std::min(P(0,0), 10000.0f);
    P(0,1) = std::min(P(0,1), 1000.0f);
    P(1,1) = std::min(P(1,1), 1000.0f);

    /*
    Calculate the predicted measurement based on the supplied alpha-beta frame
    voltage and last iteration's alpha-beta current measurement.

    ia = a * ia + b * w * sin O + c * va
    ib = a * ib - b * w * cos O + c * vb
    */
    prediction[0] =
        a_ * last_i_ab_a_[0] +
        b_est_sin_theta * state_estimate_.angular_velocity_rad_per_s +
        c_ * v_ab_v[0];
    prediction[1] =
        a_ * last_i_ab_a_[1] -
        b_est_cos_theta * state_estimate_.angular_velocity_rad_per_s +
        c_ * v_ab_v[1];

    /* Calculate innovation */
    innovation[0] = i_ab_a[0] - prediction[0];
    innovation[1] = i_ab_a[1] - prediction[1];

    /* Update the estimate */
    update[0] = K(0,0) * innovation[0] + K(1,0) * innovation[1];
    update[1] = K(0,1) * innovation[0] + K(1,1) * innovation[1];

#undef Pm
#undef P
#undef M
#undef Mt
#undef K
#undef Kt
#undef H
#undef s20
#undef s15
#undef s14
#undef s13
#undef s12
#undef s11
#undef s10
#undef s8
#undef s5
#undef s3

    /* Store the original angle */
    last_angle = state_estimate_.angle_rad;

    /*
    Get the EKF-corrected state estimate for the last PWM cycle (time t).

    While the motor is not being actively controlled (speed_setpoint == 0) try
    to estimate angle and angular velocity.
    */
    closed_loop_mul = 1.0f;
    if (closed_loop_frac < 1.0f) {
        closed_loop_mul = 0.0f;
    }
    if (speed_setpoint == 0.0f) {
        closed_loop_mul = 1.0f;
    }

    state_estimate_.angle_rad += update[1] * closed_loop_mul;
    state_estimate_.angle_rad +=
        (state_estimate_.angular_velocity_rad_per_s * t_) * closed_loop_mul;
    state_estimate_.angle_rad +=
        speed_setpoint * t_ * (1.0f - closed_loop_mul);

    /*
    Calculate filtered velocity estimate by differentiating successive
    position estimates and low-passing the result with a coefficient dependent
    on the HFI weight (higher weights = lower low-pass filter cutoff
    frequency).
    */
    acceleration = ((state_estimate_.angle_rad - last_angle) * t_inv_ -
                     state_estimate_.angular_velocity_rad_per_s);
    state_estimate_.angular_velocity_rad_per_s +=
        acceleration * angular_velocity_lpf_coeff_;

    next_angle = state_estimate_.angle_rad +
                 state_estimate_.angular_velocity_rad_per_s * t_;

    /*
    Constrain angle to +/- pi -- use conditional instructions to avoid
    branching
    */
    if (state_estimate_.angle_rad > float(M_PI)) {
        state_estimate_.angle_rad -= float(2.0 * M_PI);
    }
    if (state_estimate_.angle_rad < -float(M_PI)) {
        state_estimate_.angle_rad += float(2.0 * M_PI);
    }

    /*
    Constrain next angle to +/- pi -- use conditional instructions to avoid
    branching
    */
    if (next_angle > float(M_PI)) {
        next_angle -= float(2.0 * M_PI);
    }
    if (next_angle < -float(M_PI)) {
        next_angle += float(2.0 * M_PI);
    }

    sin_cos(next_sin_theta_, next_cos_theta_, next_angle);

    /* Track the last values for the next iteration */
    last_i_ab_a_[0] = i_ab_a[0];
    last_i_ab_a_[1] = i_ab_a[1];

    /*
    During open-loop mode (closed_loop_frac < 1.0), we estimate the value of
    phi (the back-EMF constant).
    */
    vs = __VSQRTF(v_ab_v[0] * v_ab_v[0] + v_ab_v[1] * v_ab_v[1]);
    is = __VSQRTF(i_ab_a[0] * i_ab_a[0] + i_ab_a[1] * i_ab_a[1]);

    phi = phi_estimate_v_s_per_rad_;
    if (std::abs(state_estimate_.angular_velocity_rad_per_s) > float(2.0 * M_PI) &&
            speed_setpoint != 0.0f) {
        phi = (vs - is * rs_r_) /
              std::abs(state_estimate_.angular_velocity_rad_per_s);
    }

    phi_estimate_v_s_per_rad_ +=
        (phi - phi_estimate_v_s_per_rad_) * angular_velocity_lpf_coeff_ *
        (1.0f - closed_loop_mul);
}


/*
Number of samples is selected such that the test period is almost exactly
8 full cycles of the lowest frequency (PE_START_FREQ / 8).
*/
#define PE_TEST_SAMPLES uint32_t(PE_TEST_CYCLES * (1.0f / hal_control_t_s) / \
                                                  (PE_START_FREQ_HZ / 8.0f))


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
    if (test_idx_ >= 4) {
        /* Early exit if the test is complete */
        return;
    }

    if (open_loop_test_samples_ == PE_TEST_SAMPLES) {
        /*
        Divide the Z accumulator by the number of recorded samples (half
        of the actual number of samples) to get the mean square value.
        */
        sample_z_sq_[test_idx_] *= (1.0f / float(PE_TEST_SAMPLES / 2));

        /*
        At the end of a test run, check the RMS current and increase or
        decrease voltage as necessary
        */
        if (v_ * v_ / sample_z_sq_[test_idx_] > PE_MAX_I_A * PE_MAX_I_A &&
                v_ > PE_MIN_V_V) {
            v_ *= 0.5f;
        } else if (v_ * v_ / sample_z_sq_[test_idx_] < PE_MIN_I_A * PE_MIN_I_A &&
                   v_ < PE_MAX_V_V) {
            v_ *= 2.0f;
        } else {
            /* Current was OK -- halve test frequency for the next test */
            open_loop_angular_velocity_rad_per_u_ *= 0.5f;

            /* Increment the test number */
            test_idx_++;
        }

        open_loop_test_samples_ = 0;
    } else if (open_loop_test_samples_ >= PE_TEST_SAMPLES / 2) {
        /* Accumulate the squared current and voltage readings */
        sample_z_sq_[test_idx_] +=
            (v_ab_v[0] * v_ab_v[0] + v_ab_v[1] * v_ab_v[1]) /
            (i_ab_a[0] * i_ab_a[0] + i_ab_a[1] * i_ab_a[1]);
    } else {
        sample_z_sq_[test_idx_] = 0.0f;
    }

    open_loop_test_samples_++;

    /*
    Increment the phase angle, and the cycle count if the angle has wrapped
    round to zero.
    */
    open_loop_angle_rad_ += open_loop_angular_velocity_rad_per_u_;
    if (open_loop_angle_rad_ > float(M_PI)) {
        open_loop_angle_rad_ -= float(2.0 * M_PI);
    }
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
    float w_sq, sum_xy, sum_x, sum_y, sum_x_sq, a, b;

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
        sum_xy += w_sq * sample_z_sq_[idx];
        sum_y += sample_z_sq_[idx];

        w_sq *= 0.25f; /* Halve angular velocity for the next test */
    }

    b = (4.0f * sum_xy - sum_x * sum_y) / (4.0f * sum_x_sq - sum_x * sum_x);
    a = 0.25f * (sum_y - b * sum_x);

    if (std::isnan(a) || a < 1e-6f) {
        a = 1e-6f;
    }
    r_r = __VSQRTF(a);

    if (std::isnan(b) || b < 1e-12f) {
        b = 1e-12f;
    }
    l_h = __VSQRTF(b);
}
