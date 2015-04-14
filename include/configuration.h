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

/*
64 KiB of flash:

* 8 KiB bootloader region (stubbed out in default image) at 0x08000000
* 52 KiB main application area starting at 0x08002000
* 2 KiB parameter area starting at 0x0800F000
* 2 KiB read-only area starting at 0x0800F800
*/
#define FLASH_PARAM_ADDRESS 0x0800F000
#define FLASH_PARAM_LENGTH  0x00000800

#define FLASH_READONLY_ADDRESS 0x0800F800
#define FLASH_READONLY_LENGTH  0x00000800


#include <algorithm>
#include "fixed.h"


struct bootloader_app_descriptor {
    uint64_t signature;
    uint64_t image_crc;
    uint32_t image_size;
    uint32_t vcs_commit;
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t reserved[6];
} __attribute__((packed));


/* Up to */
enum param_index_t {
    PARAM_MOTOR_NUM_POLES = 0,
    PARAM_MOTOR_CURRENT_LIMIT,
    PARAM_MOTOR_VOLTAGE_LIMIT,
    PARAM_MOTOR_RPM_MAX,
    PARAM_MOTOR_RS,
    PARAM_MOTOR_LS,
    PARAM_MOTOR_KV,
    PARAM_CONTROL_ACCEL_TORQUE_MAX,
    PARAM_CONTROL_LOAD_TORQUE,
    PARAM_CONTROL_ACCEL_GAIN,
    PARAM_CONTROL_ACCEL_TIME,
    PARAM_CUSTOM_ESCCOMMAND_ID,
    PARAM_CUSTOM_ESCSTATUS_INTERVAL,
    PARAM_CUSTOM_ESCSTATUS_ID,
    PARAM_UAVCAN_ESCSTATUS_INTERVAL,
    PARAM_UAVCAN_ESC_INDEX,
    PARAM_PWM_CONTROL_MODE,
    PARAM_PWM_THROTTLE_MIN,
    PARAM_PWM_THROTTLE_MAX,
    PARAM_PWM_THROTTLE_DEADBAND,
    PARAM_PWM_CONTROL_OFFSET,
    PARAM_PWM_CONTROL_CURVE,
    PARAM_PWM_CONTROL_MIN,
    PARAM_PWM_CONTROL_MAX,
    NUM_PARAMS
};


enum param_type_t {
    PARAM_TYPE_INT = 0,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_BOOL
};

struct param_t {
    uint8_t index;
    uint8_t public_type;
    char name[46];
    float default_value;
    float min_value;
    float max_value;
} __attribute__ ((packed));


class Configuration {
private:
    float params_[NUM_PARAMS];

public:
    Configuration(void);

    void read_motor_params(struct motor_params_t& params);
    void read_control_params(struct control_params_t& params);
    void read_pwm_params(struct pwm_params_t& params);

    bool get_param_by_name(struct param_t& out_param, const char* name);
    bool get_param_by_index(struct param_t& out_param, uint8_t index);

    bool set_param_value_by_name(const char* name, float value);
    bool set_param_value_by_index(uint8_t index, float value);

    float get_param_value_by_index(uint8_t index) {
        if (index >= NUM_PARAMS) {
            return std::numeric_limits<float>::signaling_NaN();
        } else {
            return params_[index];
        }
    }

    void write_params(void);
};
