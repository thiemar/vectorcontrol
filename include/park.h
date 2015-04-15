/*
Copyright (c) 2014 - 2015 Thiemar Pty Ltd

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

inline void __attribute__((optimize("O3")))
park_transform(
    float dq[2],
    const float alpha_beta[2],
    float sin_theta,
    float cos_theta
) {
    dq[0] = alpha_beta[0] * cos_theta + alpha_beta[1] * sin_theta;
    dq[1] = -alpha_beta[0] * sin_theta + alpha_beta[1] * cos_theta;
}


inline void __attribute__((optimize("O3")))
park_transform(
    float dq[2],
    const float alpha_beta[2],
    float theta
) {
    float sin_theta, cos_theta;

    sin_cos(sin_theta, cos_theta, theta);
    park_transform(dq, alpha_beta, sin_theta, cos_theta);
}


inline void __attribute__((optimize("O3")))
inverse_park_transform(
    float alpha_beta[2],
    const float dq[2],
    float sin_theta,
    float cos_theta
) {
    alpha_beta[0] = dq[0] * cos_theta - dq[1] * sin_theta;
    alpha_beta[1] = dq[0] * sin_theta + dq[1] * cos_theta;
}


inline void __attribute__((optimize("O3")))
inverse_park_transform(
    float alpha_beta[2],
    const float dq[2],
    float theta
) {
    float sin_theta, cos_theta;

    sin_cos(sin_theta, cos_theta, theta);
    inverse_park_transform(alpha_beta, dq, sin_theta, cos_theta);
}
