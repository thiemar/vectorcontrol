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

__attribute__ ((section(".paramflash"),used))
struct param_t flash_params[NUM_PARAMS] = {
    /*
    Index, name,
        current value, default value, min value, max value
    */

    /*
    Number of motor poles. Used to convert mechanical speeds to electrical
    speeds.
    */
    {PARAM_MOTOR_NUM_POLES, "motor_num_poles",
        14.0f, 14.0f, 4.0f, 40.0f},

    /*
    Motor current limit in amps. This determines the maximum current
    controller setpoint, as well as the maximum allowable current setpoint
    slew rate.
    */
    {PARAM_MOTOR_CURRENT_LIMIT, "motor_current_limit",
        20.0f, 10.0f, 1.0f, 40.0f},

    /*
    Motor voltage limit in volts. The current controller's commanded voltage
    will never exceed this value.

    Note that this may safely be above the nominal voltage of the motor; to
    determine the actual motor voltage limit, divide the motor's rated maximum
    power by the motor current limit.
    */
    {PARAM_MOTOR_VOLTAGE_LIMIT, "motor_voltage_limit",
        11.1f, 7.4f, 0.5f, 27.0f},

    /*
    Motor start-up current in amps. This limits the current setpoint until
    the motor reaches its minimum controllable RPM. Higher values may speed up
    initial alignment, but may also result in oscillation. Very high values
    may result in damage to the prop, motor or ESC.
    */

    {PARAM_MOTOR_STARTUP_CURRENT, "motor_startup_current",
        5.0f, 5.0f, 0.5f, 10.0f},

    /*
    Motor minimum controllable RPM. This is used to force the motor to rotate
    before the state estimation filter has locked, and also determines how
    long the motor_startup_current is used for. The value depends on the
    motor's KV as well as the number of poles; it should be low enough that
    the motor can lock onto the rotation rate immediately, but high enough
    that the back EMF is measurable.
    */
    {PARAM_MOTOR_RPM_MIN, "motor_rpm_min",
        200.0f, 130.0f, 10.0f, 1000.0f},

    /*
    Motor maximum rated RPM. This limits the upper end of the PWM setpoint
    range if it's lower than KV multiplied by Vbus.
    */
    {PARAM_MOTOR_RPM_MAX, "motor_rpm_max",
        20000.0f, 20000.0f, 500.0f, 40000.0f},

    /* Motor resistance in ohms. This is estimated on start-up. */
    {PARAM_MOTOR_RS, "motor_rs",
        60e-3f, 60e-3f, 1e-3f, 1000e-3f},

    /* Motor inductance in henries. This is estimated on start-up. */
    {PARAM_MOTOR_LS, "motor_ls",
        20e-6f, 20e-6f, 1e-6f, 1000e-6f},

    /*
    Motor KV in RPM per volt. This can be taken from the motor's spec sheet;
    accuracy will help control performance but a 20% error is fine.
    */
    {PARAM_MOTOR_KV, "motor_kv",
        850.0f, 850.0f, 100.0f, 5000.0f},

    /*
    Speed (RPM) controller gain. Determines controller aggressiveness; units
    are amp-seconds per radian. Systems with higher rotational inertia (large
    props) will need gain increased; systems with low rotational inertia
    (small props) may need gain decreased. Higher values result in faster
    response, but may result in oscillation and excessive overshoot. Lower
    values result in a slower, smoother response.
    */
    {PARAM_RPMCTL_GAIN, "rpmctl_gain",
        2.0e-2f, 15.5e-3f, 1e-6f, 1.0f},

    /*
    Speed controller bandwidth, in Hz.
    */
    {PARAM_RPMCTL_BANDWIDTH, "rpmctl_bandwidth",
        75.0f, 150.0f, 1.0f, 400.0f},

    /*
    Speed controller maximum acceleration, in rpm/s.
    */
    {PARAM_RPMCTL_ACCEL_MAX, "rpmctl_accel_max",
        30000.0f, 10000.0f, 100.0f, 100000.0f},

    /*
    Interval in seconds at which the UAVCAN standard ESC status message should
    be sent.
    */
    {PARAM_UAVCAN_ESCSTATUS_INTERVAL, "uavcan_escstatus_interval",
        10e-3f, 10e-3f, 1e-3f, 1000e-3f},

    /* Node ID of this ESC in the UAVCAN network. */
    {PARAM_UAVCAN_NODE_ID, "uavcan_node_id",
        0.0f, 0.0f, 0.0f, 125.0f},

    /* Index of this ESC in throttle command messages. */
    {PARAM_UAVCAN_ESC_INDEX, "uavcan_esc_index",
        0.0f, 0.0f, 0.0f, 15.0f},

    /*
    If 0, the PWM signal is used as the input to the speed controller, with
    input pulse width proportional to the square of the speed controller
    setpoint. If 1, the PWM signal is used as the input to the torque
    controller (the speed controller is bypassed), and the input pulse width
    is proportional to torque controller setpoint.
    */
    {PARAM_PWM_CONTROL_MODE, "pwm_ctl_mode",
        0.0f, 0.0f, 0.0f, 1.0f}
};


inline static float _rad_per_s_from_rpm(float rpm, uint32_t num_poles) {
    return rpm * 60.0f / (2.0f * (float)M_PI * (float)(num_poles >> 1u));
}


Configuration::Configuration(void) {
    memcpy(params_, flash_params, sizeof(params_));
}


void Configuration::read_motor_params(struct motor_params_t& params) {
    params.num_poles = (uint32_t)params_[PARAM_MOTOR_NUM_POLES].value;

    params.max_current_a = params_[PARAM_MOTOR_CURRENT_LIMIT].value;
    params.max_voltage_v = params_[PARAM_MOTOR_VOLTAGE_LIMIT].value;
    params.max_speed_rad_per_s =
        _rad_per_s_from_rpm(params_[PARAM_MOTOR_RPM_MAX].value,
                            params.num_poles);
    params.startup_current_a = params_[PARAM_MOTOR_STARTUP_CURRENT].value;
    params.startup_speed_rad_per_s =
        _rad_per_s_from_rpm(params_[PARAM_MOTOR_RPM_MIN].value,
                            params.num_poles);

    params.rs_r = params_[PARAM_MOTOR_RS].value;
    params.ls_h = params_[PARAM_MOTOR_LS].value;
    params.phi_v_s_per_rad =
        _rad_per_s_from_rpm(1.0f / params_[PARAM_MOTOR_KV].value,
                            params.num_poles);
}


void Configuration::read_control_params(
    struct control_params_t& params
) {
    params.gain_a_s_per_rad = params_[PARAM_RPMCTL_GAIN].value;
    params.bandwidth_hz = params_[PARAM_RPMCTL_BANDWIDTH].value;
    params.max_accel_rad_per_s2 =
        _rad_per_s_from_rpm(params_[PARAM_RPMCTL_ACCEL_MAX].value,
                            (uint32_t)params_[PARAM_MOTOR_NUM_POLES].value);
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

    idx = _find_param_index_by_name(name, params_, NUM_PARAMS);
    return get_param_by_index(out_param, (uint8_t)idx);
}


bool Configuration::get_param_by_index(
    struct param_t& out_param,
    uint8_t index
) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    memcpy(&out_param, &params_[index], sizeof(struct param_t));
    return true;
}


bool Configuration::set_param_value_by_name(const char* name, float value) {
    size_t idx;

    idx = _find_param_index_by_name(name, params_, NUM_PARAMS);
    return set_param_value_by_index((uint8_t)idx, value);
}


bool Configuration::set_param_value_by_index(uint8_t index, float value) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    if (params_[index].min_value <= value &&
            value <= params_[index].max_value) {
        params_[index].value = value;
        return true;
    } else {
        return false;
    }
}


void Configuration::write_params(void) {
    static_assert(!(sizeof(params_) & 0x3u),
                  "Size of params_ must be a multiple of 4");

    hal_flash_protect(false);
    hal_flash_erase((uint8_t*)flash_params, 2048u);
    hal_flash_write((uint8_t*)flash_params, sizeof(params_),
                    (uint8_t*)params_);
    hal_flash_protect(true);
}
