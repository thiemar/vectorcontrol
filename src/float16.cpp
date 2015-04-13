/*
This code extracted from half (http://half.sourceforge.net/)

half - IEEE 754-based half-precision floating point library.

Copyright (c) 2012-2013 Christian Rau <rauy@users.sourceforge.net>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <cstdint>
#include <cfloat>
#include <algorithm>
#include "float16.h"

float16_t __attribute__((optimize("O3")))
float16(float value) {
    uint16_t hbits;
    int32_t ival;
    int iexp;
    float diff;

    hbits = (uint16_t)(std::signbit(value) ? 0x8000u : 0);
    if (!(value < 0.0f || value > 0.0f)) {
        return hbits;
    }
    if (std::isnan(value)) {
        return hbits | 0x7FFFu;
    }
    if (std::isinf(value)) {
        return hbits | 0x7C00u;
    }
    (void)std::frexp(value, &iexp);
    if (iexp > 16) {
        return hbits | 0x7C00u;
    }
    if (iexp < -13) {
        value = std::ldexp(value, 24);
    } else {
        value = std::ldexp(value, 11 - iexp);
        hbits |= uint16_t((iexp + 14) << 10);
    }
    ival = (int32_t)value;
    hbits = (uint16_t)
        (hbits | ((uint32_t)(ival < 0 ? -ival : ival) & 0x3FFu));
    diff = std::fabs(value - (float)ival);
    hbits = (uint16_t)(hbits + (diff >= 0.5f));
    return (float16_t)hbits;
}
