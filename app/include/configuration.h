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

/*
64 KiB of flash:

* 8 KiB bootloader region (stubbed out in default image) at 0x08000000
* 52 KiB main application area starting at 0x08002000
* 4 KiB parameter area starting at 0x0800F000
*/
#define FLASH_PARAM_ADDRESS 0x0800F000
#define FLASH_PARAM_LENGTH  0x00001000


#define FLASH_PARAM_VERSION 1u


#define PARAM_NAME_MAX_LEN 46u


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
    PARAM_MOTOR_I_MAX,
    PARAM_MOTOR_V_MAX,
    PARAM_MOTOR_V_ACCEL,
    PARAM_MOTOR_RS,
    PARAM_MOTOR_LS,
    PARAM_MOTOR_KV,
    PARAM_MOTOR_INERTIA,
    PARAM_CONTROL_P_GAIN,
    PARAM_CONTROL_I_TIME,
    PARAM_CONTROL_RPM_IDLE,
    PARAM_CONTROL_SPINUP_RATE,
    PARAM_CONTROL_DIRECTION,
    PARAM_THIEMAR_STATUS_INTERVAL,
    PARAM_THIEMAR_STATUS_ID,
    PARAM_UAVCAN_ESCSTATUS_INTERVAL,
    PARAM_UAVCAN_ESC_INDEX,
    PARAM_THIEMAR_THRUSTPOWERCOMMAND_ID,
    NUM_PARAMS
};


enum param_type_t {
    PARAM_TYPE_INT = 0,
    PARAM_TYPE_FLOAT
};


struct param_t {
    uint8_t index;
    uint8_t public_type;
    const char* name;
    float default_value;
    float min_value;
    float max_value;
};


class Configuration {
private:
    float params_[NUM_PARAMS];

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
            return params_[index];
        }
    }

    void write_params(void);
};
