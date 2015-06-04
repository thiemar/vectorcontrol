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

extern volatile uint32_t g_perf_total;
extern volatile uint32_t g_perf_last;
extern volatile uint32_t g_perf_samples;

#define PERF_COUNT_START uint32_t _start_cyccnt; \
{ \
    volatile uint32_t* DWT_CYCCNT = (uint32_t*)0xE0001004; \
    asm volatile ("":::"memory"); \
    _start_cyccnt = *DWT_CYCCNT; \
    asm volatile ("":::"memory"); \
}

#define PERF_COUNT_END { \
    volatile uint32_t* DWT_CYCCNT = (uint32_t*)0xE0001004; \
    asm volatile ("":::"memory"); \
    g_perf_last = *DWT_CYCCNT - _start_cyccnt; \
    g_perf_total += g_perf_last; \
    g_perf_samples = g_perf_total < g_perf_last ? 0 : g_perf_samples + 1; \
}
