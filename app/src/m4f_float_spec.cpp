/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/float_spec.hpp>
#include <uavcan/build_config.hpp>
#include <cmath>

#if UAVCAN_USE_EXTERNAL_FLOAT16_CONVERSION
namespace uavcan
{

/*
 * IEEE754Converter
 */
uint16_t IEEE754Converter::nativeIeeeToHalf(float value)
{
    uint32_t result;
    asm ("vcvtb.f16.f32 %0, %1" : "=w" (result) : "w" (value) );
    return uint16_t(result);
}

float IEEE754Converter::halfToNativeIeee(uint16_t value)
{
    float result;
    uint32_t v = value;
    asm ("vcvtb.f32.f16 %0, %1" : "=w" (result) : "w" (v) );
    return result;
}
}
#endif // UAVCAN_USE_EXTERNAL_FLOAT16_CONVERSION
