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

#include "stm32f30x_adc.h"
#include "stm32f30x_can.h"
#include "stm32f30x_dac.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_exti.h"
#include "stm32f30x_flash.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_syscfg.h"
#include "stm32f30x_pwr.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_misc.h"

#ifdef USE_FULL_ASSERT
    #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
    void assert_failed(uint8_t* file, uint32_t line);
#else
    #define assert_param(expr) ((void)0)
#endif
