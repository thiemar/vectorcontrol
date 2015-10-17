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
#include <cstring>
#include <up_progmem.h>
#include <uavcan/data_type.hpp>

#include "configuration.h"


struct flash_param_values_t {
    uint64_t crc;
    uint32_t version;
    float values[64];
};


volatile flash_param_values_t *flash_param_values =
    (volatile flash_param_values_t*)FLASH_PARAM_ADDRESS;


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


const static struct param_t param_config_[NUM_PARAMS] = {
    /*
    Index, type, name,
        default value, min value, max value
    */

    /* Index of this ESC in throttle command messages. */
    {PARAM_UAVCAN_ESC_INDEX, PARAM_TYPE_INT, "esc_index",
        0.0f, 0.0f, 15.0f},

    /*
    Interval between publication of UAVCAN standard ESC status messages, in
    microseconds. Zero disables publication.
    */
    {PARAM_UAVCAN_ESCSTATUS_INTERVAL, PARAM_TYPE_INT, "int_status",
        50e3f, 0, 1e6f},

    /*
    Interval between publication of extended ESC status messages, in
    microseconds. Zero disables publication.
    */
    {PARAM_THIEMAR_STATUS_INTERVAL, PARAM_TYPE_INT, "int_ext_status",
        50e3, 0, 1e6f},

    /* Data type ID of extended ESC status message. */
    {PARAM_THIEMAR_STATUS_ID, PARAM_TYPE_INT, "id_ext_status",
        20034, 1, 65535},

    /*
    Number of motor poles. Used to convert mechanical speeds to electrical
    speeds.
    */
    {PARAM_MOTOR_NUM_POLES, PARAM_TYPE_INT, "mot_num_poles",
        14.0f, 2.0f, 40.0f},

    /*
    Motor current limit in amps. This determines the maximum current
    controller setpoint, as well as the maximum allowable current setpoint
    slew rate.
    */
    {PARAM_MOTOR_I_MAX, PARAM_TYPE_FLOAT, "mot_i_max",
        12.0f, 1.0f, 40.0f},

    /*
    Motor voltage limit in volts. The current controller's commanded voltage
    will never exceed this value.

    Note that this may safely be above the nominal voltage of the motor; to
    determine the actual motor voltage limit, divide the motor's rated maximum
    power by the motor current limit.
    */
    {PARAM_MOTOR_V_MAX, PARAM_TYPE_FLOAT, "mot_v_max",
        14.8f, 0.5f, 27.0f},

    /*
    Acceleration voltage limit in volts. In conjunction with the motor's
    resistance, determines the maximum acceleration torque.
    */
    {PARAM_MOTOR_V_ACCEL, PARAM_TYPE_FLOAT, "mot_v_accel",
        0.5f, 0.01f, 1.0f},

    /* Motor resistance in ohms. This is estimated on start-up. */
    {PARAM_MOTOR_RS, PARAM_TYPE_FLOAT, "mot_rs",
        1e-3f, 1e-3f, 4.0f},

    /* Motor inductance in henries. This is estimated on start-up. */
    {PARAM_MOTOR_LS, PARAM_TYPE_FLOAT, "mot_ls",
        1e-3f, 1e-6f, 1e-2f},

    /*
    Motor KV in RPM per volt. This can be taken from the motor's spec sheet;
    accuracy will help control performance but a 20% error is fine.
    */
    {PARAM_MOTOR_KV, PARAM_TYPE_FLOAT, "mot_kv",
        1000.0f, 100.0f, 8000.0f},

    /*
    Control bandwidth in hertz. Sets the input low-pass filter corner
    frequency and influences inner loop and estimator bandwidth.
    */
    {PARAM_CONTROL_BANDWIDTH, PARAM_TYPE_FLOAT, "ctl_bw",
        75.0f, 10.0f, 250.0f},

    /*
    Dimensionless control gain, used to scale the motor's measured resistance
    when determining acceleration torque.
    */
    {PARAM_CONTROL_GAIN, PARAM_TYPE_FLOAT, "ctl_gain",
        1.0f, 0.01f, 100.0f},

    /*
    If non-zero, the motor will rotate at this electrical speed in Hz when any
    command is received with a setpoint below the minimum.
    */
    {PARAM_CONTROL_HZ_IDLE, PARAM_TYPE_FLOAT, "ctl_hz_idle",
        3.5f, 0.0f, 100.0f},

    /*
    The rate at which the motor accelerates during open-loop mode, in
    electrical Hz/s.
    */
    {PARAM_CONTROL_SPINUP_RATE, PARAM_TYPE_FLOAT, "ctl_start_rate",
        25.0f, 5.0f, 1000.0f},

    /*
    Rotation direction of the motor: 0 is normal, 1 is reverse.
    */
    {PARAM_CONTROL_DIRECTION, PARAM_TYPE_INT, "ctl_dir",
        0.0f, 0.0f, 1.0f},

    /*
    Braking limit, expressed as a fraction of maximum motor current to be
    applied in negative torque. A value of 1.0 will permit symmetric
    acceleration and deceleration; a value of 0.0 will prevent active
    deceleration during normal operating modes.
    */
    {PARAM_CONTROL_BRAKING, PARAM_TYPE_FLOAT, "ctl_braking",
        1.0f, 0.0f, 1.0f}
};


Configuration::Configuration(void) {
    size_t i;
    uavcan::DataTypeSignatureCRC crc;
    crc.add((uint8_t*)(FLASH_PARAM_ADDRESS + sizeof(uint64_t)),
            sizeof(struct flash_param_values_t) - sizeof(uint64_t));

    if (crc.get() == flash_param_values->crc &&
            flash_param_values->version == FLASH_PARAM_VERSION) {
        for (i = 0; i < NUM_PARAMS; i++) {
            if (param_config_[i].min_value <= flash_param_values->values[i] &&
                flash_param_values->values[i] <= param_config_[i].max_value) {
                params_[i] = flash_param_values->values[i];
            } else {
                params_[i] = param_config_[i].default_value;
            }
        }
    } else {
        reset_params();
    }
}


void Configuration::read_motor_params(struct motor_params_t& params) {
    params.num_poles = uint32_t(params_[PARAM_MOTOR_NUM_POLES]);

    params.rs_r = params.ls_h = params.phi_v_s_per_rad = 1e-3f;

    params.accel_voltage_v = params_[PARAM_MOTOR_V_ACCEL];
    params.max_current_a = params_[PARAM_MOTOR_I_MAX];
    params.max_voltage_v = params_[PARAM_MOTOR_V_MAX];
    params.idle_speed_rad_per_s = float(2.0 * M_PI) *
                                  params_[PARAM_CONTROL_HZ_IDLE];
    params.spinup_rate_rad_per_s2 = float(2.0 * M_PI) *
                                    params_[PARAM_CONTROL_SPINUP_RATE];
}


void Configuration::read_control_params(
    struct control_params_t& params
) {
    params.bandwidth_hz = params_[PARAM_CONTROL_BANDWIDTH];
    params.gain = params_[PARAM_CONTROL_GAIN];
    params.braking_frac = params_[PARAM_CONTROL_BRAKING];
}


static size_t _find_param_index_by_name(
    const char* name,
    const struct param_t params[]
) {
    size_t i, j;

    for (i = 0; i < NUM_PARAMS; i++) {
        for (j = 0; j < PARAM_NAME_MAX_LEN &&
                    params[i].name[j] == name[j]; j++) {
            if (name[j] == 0) {
                return i;
            }
        }
    }

    return NUM_PARAMS;
}


bool Configuration::get_param_by_name(
    struct param_t& out_param,
    const char* name
) {
    size_t idx;

    idx = _find_param_index_by_name(name, param_config_);
    return get_param_by_index(out_param, (uint8_t)idx);
}


bool Configuration::get_param_by_index(
    struct param_t& out_param,
    uint8_t index
) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    out_param = param_config_[index];
    return true;
}


bool Configuration::set_param_value_by_name(const char* name, float value) {
    size_t idx;

    idx = _find_param_index_by_name(name, param_config_);
    return set_param_value_by_index((uint8_t)idx, value);
}


bool Configuration::set_param_value_by_index(uint8_t index, float value) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    if (param_config_[index].min_value <= value &&
            value <= param_config_[index].max_value) {
        params_[index] = value;
        return true;
    } else {
        return false;
    }
}


void Configuration::reset_params(void) {
    size_t i;
    for (i = 0; i < NUM_PARAMS; i++) {
        params_[i] = param_config_[i].default_value;
    }
}


void Configuration::write_params(void) {
    static_assert(!(sizeof(params_) & 0x3u),
                  "Size of params_ must be a multiple of 4");

    size_t page;
    struct flash_param_values_t new_flash_param_values;
    uavcan::DataTypeSignatureCRC crc;

    memset(&new_flash_param_values, 0xFFu,
           sizeof(struct flash_param_values_t));
    memcpy(new_flash_param_values.values, params_, sizeof(params_));
    new_flash_param_values.version = FLASH_PARAM_VERSION;

    crc.add((uint8_t*)(&new_flash_param_values.version),
            sizeof(struct flash_param_values_t) - sizeof(uint64_t));
    new_flash_param_values.crc = crc.get();

    page = up_progmem_getpage((size_t)flash_param_values);
    up_progmem_erasepage(page);
    up_progmem_write((size_t)flash_param_values, &new_flash_param_values,
                     sizeof(struct flash_param_values_t));
}
