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
64 KiB of flash, for layout see ldscripts/mem.ld:

    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 52K
    PARAMFLASH (rw) : ORIGIN = 0x0800D800, LENGTH = 4K
    BOOTLOADER (rx) : ORIGIN = 0x0800E000, LENGTH = 4K
    ROFLASH (rx) : ORIGIN = 0x0800F000, LENGTH = 4K

* 2 KiB init region with SystemInit etc (everything up to main), which will be
  updated at the end of the firmware update process
* 52 KiB main application area (main onwards)
* 2 KiB parameter area
* 4 KiB bootloader (read-only), which the init code jumps to if the bootloader
  flag is set
* 4 KiB static area (hardware ID etc, read-only)
*/

#include <algorithm>
#include "fixed.h"


enum {
    PARAM_MOTOR_NUM_POLES = 0,
    PARAM_MOTOR_CURRENT_LIMIT,
    PARAM_MOTOR_VOLTAGE_LIMIT,
    PARAM_MOTOR_STARTUP_CURRENT,
    PARAM_MOTOR_RPM_MIN,
    PARAM_MOTOR_RPM_MAX,
    PARAM_MOTOR_RS,
    PARAM_MOTOR_LS,
    PARAM_MOTOR_KV,
    PARAM_RPMCTL_GAIN,
    PARAM_RPMCTL_BANDWIDTH,
    PARAM_RPMCTL_ACCEL_MAX,
    PARAM_UAVCAN_ESCSTATUS_INTERVAL,
    PARAM_UAVCAN_NODE_ID,
    PARAM_UAVCAN_ESC_INDEX,
    PARAM_PWM_CONTROL_MODE,
    NUM_PARAMS
};


struct param_t {
    uint8_t index;
    char name[27];
    float value;
    float default_value;
    float min_value;
    float max_value;
} __attribute__ ((packed));


class Configuration {
private:
    param_t params_[NUM_PARAMS];

public:
    Configuration(void);

    void read_motor_params(struct motor_params_t& params);
    void read_control_params(struct control_params_t& params);

    bool get_param_by_name(struct param_t& out_param, const char* name);
    bool get_param_by_index(struct param_t& out_param, uint8_t index);

    bool set_param_value_by_name(const char* name, float value);
    bool set_param_value_by_index(uint8_t index, float value);

    float get_param_value_by_index(uint8_t index) {
        if (index >= NUM_PARAMS) {
            return std::numeric_limits<float>::signaling_NaN();
        } else {
            return params_[index].value;
        }
    }

    void write_params(void);
};
