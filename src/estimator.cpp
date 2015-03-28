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

#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <cstring>
#include <algorithm>
#include "stm32f30x.h"
#include "hal.h"
#include "estimator.h"
#include "perf.h"
#include "park.h"


#define STATE_DIM 2
#define MEASUREMENT_DIM 2


const float g_process_noise[2] = { 20.0f, 1e-5f };
const float g_measurement_noise[2] = { 0.005f, 0.005f };


void __attribute__((optimize("O3")))
StateEstimator::update_state_estimate(
    const float i_ab_a[2],
    const float v_ab_v[2],
    float __attribute__((unused)) accel_direction
) {
    /*
    EKF observer largely derived from:

    Smidl, V. and Peroutka, Z. (2011)
    "Reduced-Order Square-Root EKF for Sensorless Control of PMSM Drives"

    http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6119446
    */
    float covariance_temp[STATE_DIM * STATE_DIM],
          hessian[STATE_DIM * STATE_DIM],
          prediction[MEASUREMENT_DIM],
          innovation[MEASUREMENT_DIM],
          measurement_covariance[MEASUREMENT_DIM * MEASUREMENT_DIM],
          measurement_covariance_temp[MEASUREMENT_DIM * MEASUREMENT_DIM],
          kalman_gain[STATE_DIM * MEASUREMENT_DIM],
          kalman_gain_temp[STATE_DIM * MEASUREMENT_DIM],
          update[STATE_DIM * STATE_DIM], determinant, sin_theta, cos_theta,
          b_est_sin_theta, b_est_cos_theta, m_a, m_b, m_d,
          i_dq_a[2], angle_diff, hfi_scale, hfi_weight, last_angle,
          next_angle;

    /* Update the Idq estimate based on the theta estimate for time t */
    sin_cos(sin_theta, cos_theta, state_estimate_.angle_rad);
    park_transform(i_dq_a, i_ab_a, sin_theta, cos_theta);

    state_estimate_.i_dq_a[0] += (i_dq_a[0] - state_estimate_.i_dq_a[0]) *
                                 i_dq_lpf_coeff_;
    state_estimate_.i_dq_a[1] += (i_dq_a[1] - state_estimate_.i_dq_a[1]) *
                                 i_dq_lpf_coeff_;

    /*
    Highpass the current readings at the Nyquist frequency in order to extract
    the HFI signal.
    */
    i_dq_m_a_[0] = i_dq_a[0] * 0.06089863257570738f -
                   i_dq_m_a_[0] * 0.9391013674242926f;
    i_dq_m_a_[1] = i_dq_a[1] * 0.06089863257570738f -
                   i_dq_m_a_[1] * 0.9391013674242926f;

    /* Find the magnitude of the HFI signals, and lowpass that. */
    i_hfi_dq_[0] += (i_dq_m_a_[0] * i_dq_m_a_[0] - i_hfi_dq_[0]) *
                    hfi_lpf_coeff_;
    i_hfi_dq_[1] += (i_dq_m_a_[1] * i_dq_m_a_[1] - i_hfi_dq_[1]) *
                    hfi_lpf_coeff_;

    /* Store the original angle */
    last_angle = state_estimate_.angle_rad;

    /* Merge the EKF and HFI angle estimates */
    hfi_weight = get_hfi_weight();
    if (is_hfi_active_) {
        hfi_scale = std::max(0.1f, i_hfi_dq_[0] + i_hfi_dq_[1]);
        if (accel_direction > 0.0f) {
            hfi_scale = -hfi_scale;
        }
        angle_diff = (i_hfi_dq_[0] - i_hfi_dq_[1]) / hfi_scale;

        state_estimate_.angle_rad += angle_diff * hfi_weight;
    }

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
              P(1,1) + g_process_noise[1] + hfi_weight * 1e-4f;

    /* These values are used a few times */
    b_est_sin_theta = b_ * sin_theta;
    b_est_cos_theta = b_ * cos_theta;

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
    prediction[0] = a_ * last_i_ab_a_[0] +
                    b_est_sin_theta * state_estimate_.angular_velocity_rad_per_s +
                    c_ * v_ab_v[0];
    prediction[1] = a_ * last_i_ab_a_[1] -
                    b_est_cos_theta * state_estimate_.angular_velocity_rad_per_s +
                    c_ * v_ab_v[1];

    /* Calculate innovation */
    innovation[0] = i_ab_a[0] - prediction[0];
    innovation[1] = i_ab_a[1] - prediction[1];

    /* Update the estimate */
    update[0] = K(0,0) * innovation[0] + K(1,0) * innovation[1];
    update[1] = K(0,1) * innovation[0] + K(1,1) * innovation[1];

    /* Get the EKF-corrected state estimate for the last PWM cycle (time t) */
    state_estimate_.angle_rad += update[1] * std::max(0.5f, 1.0f - hfi_weight);
    state_estimate_.angle_rad +=
        state_estimate_.angular_velocity_rad_per_s * t_; //* (1.0f - hfi_weight);

    /*
    Calculate filtered velocity estimate by differentiating successive
    position estimates and low-passing the result with a coefficient dependent
    on the HFI weight (higher weights = lower low-pass filter cutoff
    frequency).
    */
    state_estimate_.angular_velocity_rad_per_s +=
        ((state_estimate_.angle_rad - last_angle) * t_inv_ -
            state_estimate_.angular_velocity_rad_per_s) * 0.01f *
        std::max(0.25f, 1.0f - hfi_weight);

    if (!is_converged()) {
        is_converged_++;
        state_estimate_.angular_velocity_rad_per_s = 0.0f;
    }

    /*
    Constrain angle to 0 .. 2 * pi; increment or decrement the revolution
    count whenever the angle exceeds that range.
    */
    if (state_estimate_.angle_rad > 2.0f * (float)M_PI) {
        state_estimate_.angle_rad -= 2.0f * (float)M_PI;
        state_estimate_.revolution_count++;
    } else if (state_estimate_.angle_rad < 0.0f) {
        state_estimate_.angle_rad += 2.0f * (float)M_PI;
        state_estimate_.revolution_count--;
    }

    next_angle = state_estimate_.angle_rad +
                 2.0f * state_estimate_.angular_velocity_rad_per_s * t_ +
                 0.1f * accel_direction * hfi_weight;
    if (next_angle > 2.0f * (float)M_PI) {
        next_angle -= 2.0f * (float)M_PI;
    } else if (next_angle < 0.0f) {
        next_angle += 2.0f * (float)M_PI;
    }

    sin_cos(next_sin_theta_, next_cos_theta_, next_angle);

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

    /* Track the last values for the next iteration */
    last_i_ab_a_[0] = i_ab_a[0];
    last_i_ab_a_[1] = i_ab_a[1];
}


#define PE_START_ANGULAR_VELOCITY 5000.0f
#define PE_START_V_V 0.25f
#define PE_MIN_V_V 0.03125f
#define PE_MAX_V_V 0.5f
#define PE_MIN_I_A 0.2f
#define PE_MAX_I_A 1.0f
#define PE_TEST_SAMPLES 1024u


void ParameterEstimator::start_estimation(float t) {
    open_loop_angle_rad_ = 0.0f;
    open_loop_test_samples_ = 0;
    open_loop_angular_velocity_rad_per_s_ = PE_START_ANGULAR_VELOCITY;
    v_ = PE_START_V_V;
    t_ = t;
    test_idx_ = 0;

    memset(sample_currents_, 0, sizeof(sample_currents_));
    memset(sample_voltages_, 0, sizeof(sample_voltages_));
}


void __attribute__((optimize("O3")))
ParameterEstimator::update_parameter_estimate(
    const float i_ab_a[2],
    const float v_ab_v[2]
) {
    /*
    Measure R and L by passing a high-frequency test signal through all three
    phases. This is the same signal as used to drive the motor, but at a
    sufficiently high frequency and low voltage that the motor doesn't rotate.

    Run four tests, each at half the frequency of the previous one.

    If the current during a test is less than the minimum, double the voltage
    and repeat (as many times as necessary). Follow the same procedure to
    increase voltage if necessary.

    At the end of the process, determine R and L by linear regression of the
    four impedance readings against the test frequencies.
    */
    float next_angle_rad;

    /* Early exit if the test is complete */
    if (test_idx_ >= 4 || v_ < PE_MIN_V_V * 0.5f) {
        return;
    }

    if (open_loop_test_samples_ == PE_TEST_SAMPLES) {
        /*
        Divide the current accumulator by the number of samples (256) to get
        the mean square value.
        */
        sample_currents_[test_idx_] *= 4.0f / (float)PE_TEST_SAMPLES;
        sample_voltages_[test_idx_] *= 4.0f / (float)PE_TEST_SAMPLES;

        /*
        At the end of a test run, check the RMS current and increase or
        decrease voltage as necessary
        */
        if (sample_currents_[test_idx_] >= PE_MAX_I_A * PE_MAX_I_A &&
                v_ > PE_MIN_V_V) {
            v_ *= 0.5f;
            sample_currents_[test_idx_] = sample_voltages_[test_idx_] = 0.0f;
        } else if (sample_currents_[test_idx_] < PE_MIN_I_A * PE_MIN_I_A &&
                   v_ < PE_MAX_V_V) {
            v_ *= 2.0f;
            sample_currents_[test_idx_] = sample_voltages_[test_idx_] = 0.0f;
        } else {
            /*
            Current was OK -- halve test voltage, test frequency and test
            cycle count for the next test.
            */
            open_loop_angular_velocity_rad_per_s_ *= 0.5f;

            /* Increment the test number */
            test_idx_++;
        }

        open_loop_test_samples_ = 0;
        open_loop_angle_rad_ = 0.0f;
    } else if (open_loop_test_samples_ >= 3 * PE_TEST_SAMPLES / 4) {
        /* Accumulate the squared current and voltage readings */
        sample_currents_[test_idx_] +=
            (i_ab_a[0] * i_ab_a[0] + i_ab_a[1] * i_ab_a[1]);
        sample_voltages_[test_idx_] +=
            (v_ab_v[0] * v_ab_v[0] + v_ab_v[1] * v_ab_v[1]);
    }

    /*
    Increment the phase angle, and the cycle count if the angle has wrapped
    round to zero.
    */
    next_angle_rad = open_loop_angle_rad_ +
                     open_loop_angular_velocity_rad_per_s_ * t_;
    if (next_angle_rad > 2.0f * (float)M_PI) {
        next_angle_rad -= 2.0f * (float)M_PI;
    }
    open_loop_angle_rad_ = next_angle_rad;
    open_loop_test_samples_++;
}


void __attribute__((optimize("O3")))
ParameterEstimator::get_v_alpha_beta_v(float v_ab_v[2]) {
    /*
    Get alpha and beta voltage components for the latest phase angle. Only
    output a voltage for the first 12 out of 16 test cycles to give time for
    the current to decay to zero.
    */
    float v_dq[2];

    v_dq[0] = test_idx_ < 4 ? v_ : 0.0f;
    v_dq[1] = 0.0f;

    inverse_park_transform(v_ab_v, v_dq, open_loop_angle_rad_);
}


void ParameterEstimator::calculate_r_l_from_samples(
    float& r_r,
    float& l_h,
    const float v_sq[4],
    const float i_sq[4]
) {
    size_t idx;
    float z_sq, w_sq, sum_xy, sum_x, sum_y, sum_x_sq, a, b;

    sum_xy = sum_x = sum_y = sum_x_sq = 0;

    /*
    Run simple linear regression, and calculate a and b such that
    y = bx + a

    where y = z^2 and x = w^2.

    L is then sqrt(b) and R is sqrt(a).
    */
    for (idx = 0; idx < 4; idx++) {
        z_sq = v_sq[idx] / i_sq[idx];
        w_sq = PE_START_ANGULAR_VELOCITY / (float)(1 << idx);
        w_sq *= w_sq;

        sum_xy += w_sq * z_sq;
        sum_x += w_sq;
        sum_y += z_sq;
        sum_x_sq += w_sq * w_sq;
    }

    b = (4.0f * sum_xy - sum_x * sum_y) / (4.0f * sum_x_sq - sum_x * sum_x);
    a = 0.25f * (sum_y - b * sum_x);

    if (a < 0.0f) {
        a = 0.0f;
    }

    r_r = std::max(__VSQRTF(a * 0.25f), 0.01f);
    l_h = std::max(__VSQRTF(b * 0.25f), 10e-6f);
}
