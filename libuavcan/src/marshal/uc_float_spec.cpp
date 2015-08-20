/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/float_spec.hpp>
#include <uavcan/build_config.hpp>
#include <cmath>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <limits>
#endif

namespace uavcan
{
/*
 * IEEE754Converter
 */
uint16_t IEEE754Converter::nativeNonIeeeToHalf(float value)
{
#if UAVCAN_TINY
    /*
    https://github.com/numpy/numpy/blob/master/numpy/core/src/npymath/halffloat.c
    BSD license
    */
    union { float val; uint32_t valbits; } conv;
    uint32_t f_exp, f_sig;
    uint16_t h_sgn, h_exp, h_sig;

    conv.val = value;

    h_sgn = (uint16_t)((conv.valbits & 0x80000000U) >> 16);
    f_exp = (conv.valbits & 0x7F800000U);

    /* Exponent overflow/NaN converts to signed inf/NaN */
    if (f_exp >= 0x47800000U) {
        if (f_exp == 0x7F800000U) {
            /* Inf or NaN */
            f_sig = (conv.valbits & 0x007FFFFFU);
            if (f_sig != 0) {
                /* NaN - propagate the flag in the significand... */
                uint16_t ret = (uint16_t)(0x7C00U + (f_sig >> 13));
                /* ...but make sure it stays a NaN */
                if (ret == 0x7C00U) {
                    ret++;
                }
                return h_sgn + ret;
            } else {
                /* signed inf */
                return (uint16_t)(h_sgn + 0x7C00U);
            }
        } else {
            /* overflow to signed inf */
            return (uint16_t)(h_sgn + 0x7C00U);
        }
    }

    /* Exponent underflow converts to a subnormal half or signed zero */
    if (f_exp <= 0x38000000U) {
        /*
         * Signed zeros, subnormal floats, and floats with small
         * exponents all convert to signed zero halfs.
         */
        if (f_exp < 0x33000000U) {
            return h_sgn;
        }

        /* Make the subnormal significand */
        f_exp >>= 23;
        f_sig = (0x00800000U + (conv.valbits & 0x007FFFFFU));
        f_sig >>= (113 - f_exp);
        /* Handle rounding by adding 1 to the bit beyond half precision */
        f_sig += 0x00001000U;

        h_sig = (uint16_t)(f_sig >> 13);

        /*
         * If the rounding causes a bit to spill into h_exp, it will
         * increment h_exp from zero to one and h_sig will be zero.
         * This is the correct result.
         */
        return (uint16_t)(h_sgn + h_sig);
    }

    /* Regular case with no overflow or underflow */
    h_exp = (uint16_t)((f_exp - 0x38000000U) >> 13);

    /* Handle rounding by adding 1 to the bit beyond half precision */
    f_sig = (conv.valbits & 0x007FFFFFU);
    f_sig += 0x00001000U;

    h_sig = (uint16_t)(f_sig >> 13);

    /*
     * If the rounding causes a bit to spill into h_exp, it will
     * increment h_exp by one and h_sig will be zero.  This is the
     * correct result.  h_exp may increment to 15, at greatest, in
     * which case the result overflows to a signed inf.
     */
    return h_sgn + h_exp + h_sig;
#else
    /* http://half.sourceforge.net/ (MIT License) */
    uint16_t hbits = uint16_t(getSignBit(value) ? 0x8000U : 0);
    if (areFloatsExactlyEqual(value, 0.0F))
    {
        return hbits;
    }
    if (isNaN(value))
    {
        return hbits | 0x7FFFU;
    }
    if (isInfinity(value))
    {
        return hbits | 0x7C00U;
    }
    int exp;
    (void)std::frexp(value, &exp);
    if (exp > 16)
    {
        return hbits | 0x7C00U;
    }
    if (exp < -13)
    {
        value = std::ldexp(value, 24);
    }
    else
    {
        value = std::ldexp(value, 11 - exp);
        hbits |= uint16_t((exp + 14) << 10);
    }
    const int32_t ival = static_cast<int32_t>(value);
    hbits = uint16_t(hbits | (uint32_t((ival < 0) ? (-ival) : ival) & 0x3FFU));
    float diff = std::fabs(value - static_cast<float>(ival));
    hbits = uint16_t(hbits + (diff >= 0.5F));
    return hbits;
#endif // UAVCAN_TINY
}

float IEEE754Converter::halfToNativeNonIeee(uint16_t value)
{
#if UAVCAN_TINY
    /*
    https://github.com/numpy/numpy/blob/master/numpy/core/src/npymath/halffloat.c
    BSD license
    */
    union { float ret; uint32_t retbits; } conv;

    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    h_exp = (value & 0x7C00U);
    f_sgn = ((uint32_t)value & 0x8000U) << 16;
    switch (h_exp) {
        case 0x0000U: /* 0 or subnormal */
            h_sig = (value & 0x03FFU);
            if (h_sig == 0) {
                /* Signed zero */
                conv.retbits = f_sgn;
            } else {
                /* Subnormal */
                h_sig <<= 1;
                while ((h_sig & 0x0400U) == 0) {
                    h_sig <<= 1;
                    h_exp++;
                }
                f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
                f_sig = ((uint32_t)(h_sig & 0x03FFU)) << 13;
                conv.retbits = f_sgn + f_exp + f_sig;
            }
            break;
        case 0x7C00U: /* inf or NaN */
            /* All-ones exponent and a copy of the significand */
            conv.retbits = f_sgn + 0x7F800000U + (((uint32_t)(value & 0x03FFU)) << 13);
            break;
        default: /* normalized */
            /* Just need to adjust the exponent and shift */
            conv.retbits = f_sgn + (((uint32_t)(value & 0x7FFFU) + 0x1C000U) << 13);
            break;
    }
    return conv.ret;
#else
    /* http://half.sourceforge.net/ (MIT License) */
    float out;
    unsigned abs = value & 0x7FFFU;
    if (abs > 0x7C00U)
    {
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        out = std::numeric_limits<float>::has_quiet_NaN ? std::numeric_limits<float>::quiet_NaN() : 0.0F;
#else
        out = nanf("");
#endif
    }
    else if (abs == 0x7C00U)
    {
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        out = std::numeric_limits<float>::has_infinity ?
              std::numeric_limits<float>::infinity() : std::numeric_limits<float>::max();
#else
        out = NumericTraits<float>::infinity();
#endif
    }
    else if (abs > 0x3FFU)
    {
        out = std::ldexp(static_cast<float>((value & 0x3FFU) | 0x400U), int(abs >> 10) - 25);
    }
    else
    {
        out = std::ldexp(static_cast<float>(abs), -24);
    }
    return (value & 0x8000U) ? -out : out;
#endif // UAVCAN_TINY
}

}
