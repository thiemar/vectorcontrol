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
