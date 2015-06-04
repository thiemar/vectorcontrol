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


enum hal_status_t {
    HAL_STATUS_OK = 0,
    HAL_STATUS_ERROR = 1
};


enum hal_pwm_state_t {
	HAL_PWM_STATE_OFF = 0,
	HAL_PWM_STATE_LOW = 1,
	HAL_PWM_STATE_RUNNING = 2
};


typedef void (*hal_callback_t)(void);
/* Called with output Vab, last output Vab, last Iab, and Vbus */
typedef void (*hal_control_callback_t)(float*, const float*,
                                       const float*, float);
typedef void (*hal_pwm_callback_t)(uint32_t, uint32_t);


extern const uint32_t hal_core_frequency_hz;
extern const uint32_t hal_pwm_frequency_hz;
extern const float hal_full_scale_current_a;
extern const float hal_full_scale_voltage_v;
extern const float hal_control_t_s;


/* Public interface */
void hal_reset(void);
void hal_set_pwm_state(enum hal_pwm_state_t state);
enum hal_status_t hal_transmit_can_message(
    uint8_t mailbox,
    uint32_t message_id,
    size_t length,
    const uint8_t *message
);
enum hal_status_t hal_receive_can_message(
    uint8_t fifo,
    uint8_t *filter_id,
    uint32_t *message_id,
    size_t *length,
    uint8_t *message
);
void hal_set_can_dtid_filter(
    uint8_t fifo,
    uint8_t filter_id,
    uint8_t transfer_type,
    uint16_t dtid
);
void hal_disable_can_transmit(void);
void hal_enable_can_transmit(void);
float hal_get_temperature_degc(void);
void hal_set_low_frequency_callback(hal_callback_t callback);
void hal_set_high_frequency_callback(hal_control_callback_t callback);
void hal_set_rc_pwm_callback(hal_pwm_callback_t callback);
void hal_flash_protect(bool readonly);
void hal_flash_erase(uint8_t *addr, size_t len);
void hal_flash_write(uint8_t *addr, size_t len, const uint8_t *data);
void hal_restart(void);
