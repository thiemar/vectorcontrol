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
#include <stm32.h>

#include <irq.h>
#include "hal.h"
#include "can.h"
#include "svm.h"
#include "perf.h"
#include "shared.h"


/* Hardware states */
static volatile enum hal_pwm_state_t pwm_state_;


/* Low-frequency (SysTick) control task callback */
static volatile hal_callback_t low_frequency_task_;
/* High-frequency (ADC update) control task callback */
static volatile hal_control_callback_t high_frequency_task_;


/* Bootloader parameters */
static struct bootloader_app_shared_t bootloader_app_shared_;


/* ADC measurements */
static volatile float vbus_v_;
static volatile float temp_degc_;


/* If true, phases A and B are swapped to reverse the directon of the motor */
static volatile bool phase_reverse_;


static void hal_init_sys_();
static void hal_init_tim_();
static void hal_init_adc_();
static void hal_run_calibration_();
static bool hal_adc_periodic_();


void
__attribute__((noreturn))
__esc_assert_func (
    const char __attribute__((unused)) *file,
    int __attribute__((unused)) line,
    const char __attribute__((unused)) *func,
    const char __attribute__((unused)) *failedexpr
) {
    hal_set_pwm_state(HAL_PWM_STATE_LOW);
    while (1);
}


extern "C" void sched_process_timer(void) {
    /* Dummy function */
}


extern "C" void
__attribute__((noreturn))
default_handler(void) {
    hal_set_pwm_state(HAL_PWM_STATE_LOW);
    while (1);
}


extern "C" void stm32_systick(void) {
    /*
    Get the latest bus voltage or temperature, depending on what was requested
    */
    hal_adc_periodic_();

    /* Run the user task, if defined */
    if (low_frequency_task_) {
        low_frequency_task_();
    }
}


static void hal_init_sys_() {
    /* Enable DWT count-down */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


uint8_t hal_get_can_node_id(void) {
    return (uint8_t)(bootloader_app_shared_.node_id & 0x7Fu);
}


float hal_get_temperature_degc(void) {
    return temp_degc_;
}


void hal_set_low_frequency_callback(hal_callback_t callback) {
    esc_assert(!callback || !low_frequency_task_);
    low_frequency_task_ = callback;
}


void hal_set_high_frequency_callback(hal_control_callback_t callback) {
    esc_assert(!callback || !high_frequency_task_);
    high_frequency_task_ = callback;
}


void hal_set_pwm_reverse(bool reverse) {
    if (pwm_state_ != HAL_PWM_STATE_RUNNING) {
        phase_reverse_ = reverse;
    }
}


void hal_restart(void) {
    hal_set_pwm_state(HAL_PWM_STATE_LOW);
    up_systemreset();
}


#if defined(S2740VC_1_0)
    #include "hal_s2740vc_1_0.cpp"
#elif defined(PX4ESC_1_6)
    #include "hal_px4esc_1_6.cpp"
#endif /* S2740VC_1_0 */
