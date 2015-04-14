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

#include <cstdlib>
#include <cstring>

#include "configuration.h"
#include "hal.h"


volatile float *flash_param_values = (volatile float*)FLASH_PARAM_ADDRESS;


__attribute__((section(".app_descriptor"),used))
volatile struct bootloader_app_descriptor
flash_app_descriptor = {
    .signature = 0x3030637365445041L,
    .image_crc = 0L,
    .image_size = 0L,
    .vcs_commit = 0L,
    .major_version = 0u,
    .minor_version = 1u,
    .reserved = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu}
};


struct param_t flash_params[NUM_PARAMS] = {
    /*
    Index, type, name,
        default value, min value, max value
    */

    /*
    Number of motor poles. Used to convert mechanical speeds to electrical
    speeds.
    */
    {PARAM_MOTOR_NUM_POLES, PARAM_TYPE_INT, "motor_num_poles",
        14.0f, 2.0f, 40.0f},

    /*
    Motor current limit in amps. This determines the maximum current
    controller setpoint, as well as the maximum allowable current setpoint
    slew rate.
    */
    {PARAM_MOTOR_CURRENT_LIMIT, PARAM_TYPE_FLOAT, "motor_current_limit",
        10.0f, 1.0f, 40.0f},

    /*
    Motor voltage limit in volts. The current controller's commanded voltage
    will never exceed this value.

    Note that this may safely be above the nominal voltage of the motor; to
    determine the actual motor voltage limit, divide the motor's rated maximum
    power by the motor current limit.
    */
    {PARAM_MOTOR_VOLTAGE_LIMIT, PARAM_TYPE_FLOAT, "motor_voltage_limit",
        7.4f, 0.5f, 27.0f},

    /*
    Motor maximum rated RPM. This limits the upper end of the PWM setpoint
    range if it's lower than KV multiplied by Vbus.
    */
    {PARAM_MOTOR_RPM_MAX, PARAM_TYPE_INT, "motor_rpm_max",
        20000.0f, 500.0f, 40000.0f},

    /* Motor resistance in ohms. This is estimated on start-up. */
    {PARAM_MOTOR_RS, PARAM_TYPE_FLOAT, "motor_rs",
        60e-3f, 1e-3f, 1000e-3f},

    /* Motor inductance in henries. This is estimated on start-up. */
    {PARAM_MOTOR_LS, PARAM_TYPE_FLOAT, "motor_ls",
        20e-6f, 1e-6f, 1000e-6f},

    /*
    Motor KV in RPM per volt. This can be taken from the motor's spec sheet;
    accuracy will help control performance but a 20% error is fine.
    */
    {PARAM_MOTOR_KV, PARAM_TYPE_FLOAT, "motor_kv",
        850.0f, 100.0f, 5000.0f},

    /*
    Acceleration torque limit in amps. Determines the maximum difference
    between the torque setpoint and the load torque, and therefore the amount
    of torque available for acceleration.

    This is a critical factor in smooth start-up into high-inertia systems. If
    start-up is rough, lower this parameter and/or control_accel_gain. If
    controller response is too slow, increase this parameter and/or
    control_accel_gain.
    */
    {PARAM_CONTROL_ACCEL_TORQUE_MAX, PARAM_TYPE_FLOAT, "control_accel_torque_max",
        2.0f, 0.1f, 40.0f},

    /*
    Load torque in amps. This is a target value for torque at full throttle,
    rather than a limit, and in conjunction with control_accel_time it
    determines the torque output from the speed controller.
    */
    {PARAM_CONTROL_LOAD_TORQUE, PARAM_TYPE_FLOAT, "control_load_torque",
        10.0f, 1.0f, 40.0f},

    /*
    Speed controller acceleration gain. A gain of 0.0 results in no torque
    output proportional to the required acceleration, whiel a gain of 1.0
    results in a full-scale acceleration torque output for an error of
    100 rad/s electrical.
    */
    {PARAM_CONTROL_ACCEL_GAIN, PARAM_TYPE_FLOAT, "control_accel_gain",
        0.1f, 0.0f, 1.0f},

    /*
    Rise time of the speed controller's torque output; this determines the
    target time to accelerate from near zero to full throttle, subject to
    the overall current limits and load inertia.
    */
    {PARAM_CONTROL_ACCEL_TIME, PARAM_TYPE_FLOAT, "control_accel_time",
        0.1f, 0.01f, 1.0f},

    /* Data type ID of the custom ESC status message. */
    {PARAM_CUSTOM_ESCCOMMAND_ID, PARAM_TYPE_INT,
        "uavcan.dtid-thiemar.equipment.esc.Command",
        700, 1, 2047},

    /*
    Interval in microseconds at which custom ESC status messages should be
    sent. Zero disables publication.
    */
    {PARAM_CUSTOM_ESCSTATUS_INTERVAL, PARAM_TYPE_INT,
        "uavcan.pubp-thiemar.equipment.esc.Status",
        20e3, 0, 1e6f},

    /* Data type ID of the custom ESC status message. */
    {PARAM_CUSTOM_ESCSTATUS_ID, PARAM_TYPE_INT,
        "uavcan.dtid-thiemar.equipment.esc.Status",
        700, 1, 2047},

    /*
    Interval in microseconds at which UAVCAN standard ESC status messages
    should be sent. Zero disables publication.
    */
    {PARAM_UAVCAN_ESCSTATUS_INTERVAL, PARAM_TYPE_INT,
        "uavcan.pubp-uavcan.equipment.esc.Status",
        1e5f, 0, 1e6f},

    /* Index of this ESC in throttle command messages. */
    {PARAM_UAVCAN_ESC_INDEX, PARAM_TYPE_INT,
        "uavcan.id-uavcan.equipment.esc-esc_index",
        0.0f, 0.0f, 15.0f},

    /*
    If 0, the PWM signal is used as the input to the speed controller, with
    input pulse width proportional to the square of the speed controller
    setpoint. If 1, the PWM signal is used as the input to the torque
    controller (the speed controller is bypassed), and the input pulse width
    is proportional to torque controller setpoint.
    */
    {PARAM_PWM_CONTROL_MODE, PARAM_TYPE_INT, "pwm_control_mode",
        0.0f, 0.0f, 1.0f},

    /*
    Sets the pulse width (in us) at which the controller is activated and the
    output setpoint takes its minimum value.
    */
    {PARAM_PWM_THROTTLE_MIN, PARAM_TYPE_INT, "pwm_throttle_min",
        1100.0f, 1000.0f, 2000.0f},

    /*
    Sets the pulse width (in us) at which the output setpoint takes its
    maximum value.
    */
    {PARAM_PWM_THROTTLE_MAX, PARAM_TYPE_INT, "pwm_throttle_max",
        1900.0f, 1000.0f, 2000.0f},

    /*
    Sets the range of pulse widths (in us) either side of the zero throttle
    point within which the output setpoint should be pwm_control_min.
    */
    {PARAM_PWM_THROTTLE_DEADBAND, PARAM_TYPE_INT, "pwm_throttle_deadband",
        10.0f, 0.0f, 1000.0f},

    /*
    Offsets the range of output setpoint values.

    If the value of this parameter is 0.5, the range of setpoints output from
    [pwm_throttle_min, pwm_throttle_max] is [-pwm_control_max, pwm_control_max].

    If the value is 0.0, the range of setpoints output from
    [pwm_throttle_min, pwm_throttle_max] is [pwm_control_min, pwm_control_max].
    */
    {PARAM_PWM_CONTROL_OFFSET, PARAM_TYPE_FLOAT, "pwm_control_offset",
        0.0f, -1.0f, 1.0f},

    /*
    Determines the relationship between input throttle and output setpoint.
    Valide values are 0.5, which results in the setpoint being proportional
    to the square root of the throttle; 1.0, which results in a linear
    relationship between setpoint and throttle; and 2.0, which results in the
    setpoint being proportional to the square of the throttle.
    */
    {PARAM_PWM_CONTROL_CURVE, PARAM_TYPE_FLOAT, "pwm_control_curve",
        1.0f, 0.5f, 2.0f},

    /* Determines the setpoint for the minimum valid throttle value. */
    {PARAM_PWM_CONTROL_MIN, PARAM_TYPE_FLOAT, "pwm_control_min",
        0.0f, 0.0f, 40000.0f},

    /* Determines the setpoint for the maximum valid throttle value. */
    {PARAM_PWM_CONTROL_MAX, PARAM_TYPE_FLOAT, "pwm_control_max",
        0.0f, 0.0f, 40000.0f},
};


inline static float _rad_per_s_from_rpm(float rpm, uint32_t num_poles) {
    return rpm * 60.0f / (2.0f * (float)M_PI * (float)(num_poles >> 1u));
}


Configuration::Configuration(void) {
    size_t i;
    for (i = 0; i < NUM_PARAMS; i++) {
        params_[i] = flash_param_values[i];
    }
}


void Configuration::read_motor_params(struct motor_params_t& params) {
    params.num_poles = (uint32_t)params_[PARAM_MOTOR_NUM_POLES];

    params.max_current_a = params_[PARAM_MOTOR_CURRENT_LIMIT];
    params.max_voltage_v = params_[PARAM_MOTOR_VOLTAGE_LIMIT];
    params.max_speed_rad_per_s =
        _rad_per_s_from_rpm(params_[PARAM_MOTOR_RPM_MAX], params.num_poles);

    params.rs_r = params_[PARAM_MOTOR_RS];
    params.ls_h = params_[PARAM_MOTOR_LS];
    params.phi_v_s_per_rad =
        _rad_per_s_from_rpm(1.0f / params_[PARAM_MOTOR_KV], params.num_poles);
}


void Configuration::read_control_params(
    struct control_params_t& params
) {
    params.bandwidth_hz = 50.0f;
    params.max_accel_torque_a = params_[PARAM_CONTROL_ACCEL_TORQUE_MAX];
    params.load_torque_a = params_[PARAM_CONTROL_LOAD_TORQUE];
    params.accel_gain = params_[PARAM_CONTROL_ACCEL_GAIN];
    params.accel_time_s = params_[PARAM_CONTROL_ACCEL_TIME];
}


void Configuration::read_pwm_params(struct pwm_params_t& params) {
    params.use_speed_controller =
        params_[PARAM_PWM_CONTROL_MODE] > 0.0f ? false : true;
    params.throttle_pulse_min_us = (uint16_t)params_[PARAM_PWM_THROTTLE_MIN];
    params.throttle_pulse_max_us = (uint16_t)params_[PARAM_PWM_THROTTLE_MAX];
    params.throttle_deadband_us =
        (uint16_t)params_[PARAM_PWM_THROTTLE_DEADBAND];
    params.control_offset = params_[PARAM_PWM_CONTROL_OFFSET];
    params.control_min = params_[PARAM_PWM_CONTROL_MIN];
    params.control_max = params_[PARAM_PWM_CONTROL_MAX];
    switch ((uint8_t)params_[PARAM_PWM_CONTROL_CURVE]) {
        case 0:
            params.control_curve = pwm_params_t::SQRT;
            break;
        case 1:
            params.control_curve = pwm_params_t::LINEAR;
            break;
        case 2:
            params.control_curve = pwm_params_t::QUADRATIC;
            break;
        default:
            params.control_curve = pwm_params_t::LINEAR;
            break;
    }
}


static size_t _get_param_name_len(const char* name) {
    size_t i;

    for (i = 0; i < 27; i++) {
        if (name[i] == 0) {
            return i + 1u;
        }
    }

    return 0;
}


static size_t _find_param_index_by_name(
    const char* name,
    struct param_t params[],
    size_t num_params
) {
    size_t i, name_len;

    name_len = _get_param_name_len(name);
    if (name_len <= 1) {
        return num_params;
    }

    for (i = 0; i < num_params; i++) {
        if (memcmp(params[i].name, name, name_len) == 0) {
            return i;
        }
    }

    return num_params;
}


bool Configuration::get_param_by_name(
    struct param_t& out_param,
    const char* name
) {
    size_t idx;

    idx = _find_param_index_by_name(name, flash_params, NUM_PARAMS);
    return get_param_by_index(out_param, (uint8_t)idx);
}


bool Configuration::get_param_by_index(
    struct param_t& out_param,
    uint8_t index
) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    memcpy(&out_param, &flash_params[index], sizeof(struct param_t));
    return true;
}


bool Configuration::set_param_value_by_name(const char* name, float value) {
    size_t idx;

    idx = _find_param_index_by_name(name, flash_params, NUM_PARAMS);
    return set_param_value_by_index((uint8_t)idx, value);
}


bool Configuration::set_param_value_by_index(uint8_t index, float value) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    if (flash_params[index].min_value <= value &&
            value <= flash_params[index].max_value) {
        params_[index] = value;
        return true;
    } else {
        return false;
    }
}


void Configuration::write_params(void) {
    static_assert(!(sizeof(params_) & 0x3u),
                  "Size of params_ must be a multiple of 4");

    hal_flash_protect(false);
    hal_flash_erase((uint8_t*)flash_param_values, 2048u);
    hal_flash_write((uint8_t*)flash_param_values, sizeof(params_),
                    (uint8_t*)params_);
    hal_flash_protect(true);
}
