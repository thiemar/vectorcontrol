/*
Copyright (c) 2014 - 2015 by Thiemar Pty Ltd

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
void hal_set_can_dtid_filter(uint8_t fifo, uint8_t filter_id, uint16_t dtid);
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
