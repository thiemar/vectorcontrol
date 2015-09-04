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

inline void __attribute__((always_inline))
park_transform(
    float dq[2],
    const float alpha_beta[2],
    float sin_theta,
    float cos_theta
) {
    dq[0] = alpha_beta[0] * cos_theta + alpha_beta[1] * sin_theta;
    dq[1] = -alpha_beta[0] * sin_theta + alpha_beta[1] * cos_theta;
}


inline void __attribute__((always_inline))
park_transform(
    float dq[2],
    const float alpha_beta[2],
    float theta
) {
    float sin_theta, cos_theta;

    sin_cos(sin_theta, cos_theta, theta);
    park_transform(dq, alpha_beta, sin_theta, cos_theta);
}


inline void __attribute__((always_inline))
inverse_park_transform(
    float alpha_beta[2],
    const float dq[2],
    float sin_theta,
    float cos_theta
) {
    alpha_beta[0] = dq[0] * cos_theta - dq[1] * sin_theta;
    alpha_beta[1] = dq[0] * sin_theta + dq[1] * cos_theta;
}


inline void __attribute__((always_inline))
inverse_park_transform(
    float alpha_beta[2],
    const float dq[2],
    float theta
) {
    float sin_theta, cos_theta;

    sin_cos(sin_theta, cos_theta, theta);
    inverse_park_transform(alpha_beta, dq, sin_theta, cos_theta);
}
