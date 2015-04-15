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
#include <cstdint>
#include <cassert>
#include <cmath>
#include <algorithm>
#include "stm32f30x.h"
#include "core_cmInstr.h"
#include "hal.h"
#include "park.h"
#include "perf.h"
#include "estimator.h"
#include "controller.h"
#include "configuration.h"
#include "uavcan.h"


/*
"Owned" by identification_cb / control_cb (only one will be running at a given
stage in execution).
*/
static StateEstimator g_estimator;
static ParameterEstimator g_parameter_estimator;
static DQCurrentController g_current_controller;
static SpeedController g_speed_controller;
static AlignmentController g_alignment_controller;


enum controller_mode_t {
    CONTROLLER_STOPPED,
    CONTROLLER_IDENTIFYING,
    CONTROLLER_SPEED,
    CONTROLLER_TORQUE,
    CONTROLLER_STOPPING
};

struct controller_state_t {
    uint32_t time;
    uint32_t last_setpoint_update;
    float current_setpoint;
    float speed_setpoint;
    enum controller_mode_t mode;
    bool fault;
};


/*
Globally-visible board state updated by high-frequency callbacks and read by
the low-frequency callback (doesn't matter if it's slightly out of sync as
it's only used for reporting).
*/
volatile static struct controller_state_t g_controller_state;
volatile static struct motor_state_t g_motor_state;

volatile static uint32_t g_pwm_width_us, g_pwm_period_us;
volatile static bool g_pwm_updated;

volatile static float g_vbus_v;
volatile static float g_v_dq_v[2];

volatile static float g_i_samples[4];
volatile static float g_v_samples[4];
volatile static bool g_parameter_estimator_done;
volatile static float g_estimator_consistency;
volatile static float g_estimator_hfi[2];


/* Written to the image in post-processing */
extern volatile struct bootloader_app_descriptor flash_app_descriptor;


/* Timeouts */
#define INTERFACE_TIMEOUT_MS 1500u
#define THROTTLE_TIMEOUT_MS 500u


/* Hard current limit -- exceeding this shuts down the motor */
#define HARD_CURRENT_LIMIT_A 60.0f


/* RC PWM minimum and maximum limits */
#define RC_PWM_ARM_PULSES 10u
#define RC_PWM_MIN_WIDTH_US 950u
#define RC_PWM_MAX_WIDTH_US 2100u
#define RC_PWM_MIN_PERIOD_US 2000u
#define RC_PWM_MAX_PERIOD_US 50000u


extern "C" int main(void);
extern "C" void identification_cb(
    float out_v_ab_v[2],
    const float last_v_ab_v[2],
    const float i_ab_a[2],
    float vbus_v
);
extern "C" void control_cb(
    float out_v_ab_v[2],
    const float last_v_ab_v[2],
    const float i_ab_a[2],
    float vbus_v
);
extern "C" void systick_cb(void);
extern "C" bool can_restart_command_cb(void);
extern "C" bool can_write_flash_command_cb(void);
extern "C" void rc_pwm_cb(uint32_t width_us, uint32_t period_us);

static void node_init(void);
static void node_run(
    Configuration& configuration,
    const struct motor_params_t& motor_params,
    const struct pwm_params_t& pwm_params
);
static float pwm_calculate_setpoint(
    const struct pwm_params_t& params,
    uint32_t width_us
);


void __attribute__((optimize("O3")))
identification_cb(
    float out_v_ab_v[2],
    const float last_v_ab_v[2],
    const float i_ab_a[2],
    float vbus_v
) {
    float i_samples[4], v_samples[4];
    size_t i;

    /* Update the parameter estimate with those values */
    g_parameter_estimator.update_parameter_estimate(i_ab_a,
                                                    last_v_ab_v);
    g_parameter_estimator.get_v_alpha_beta_v(out_v_ab_v);

    if (g_parameter_estimator.is_estimation_complete()) {
        g_parameter_estimator.get_samples(v_samples, i_samples);

        for (i = 0; i < 4; i++) {
            g_i_samples[i] = i_samples[i];
            g_v_samples[i] = v_samples[i];
        }
        g_parameter_estimator_done = true;
    }

    /*
    Handle stop condition -- set Vd, Vq to zero. Trigger at current limit.
    */
    if (i_ab_a[0] * i_ab_a[0] + i_ab_a[1] * i_ab_a[1] >
            HARD_CURRENT_LIMIT_A * HARD_CURRENT_LIMIT_A ||
            g_controller_state.fault) {
        g_controller_state.fault = true;
        g_controller_state.mode = CONTROLLER_STOPPED;
    }
    if (g_controller_state.mode == CONTROLLER_STOPPED) {
        out_v_ab_v[0] = out_v_ab_v[1] = 0.0f;
    }

    g_vbus_v = vbus_v;
}


void __attribute__((optimize("O3")))
control_cb(
    float out_v_ab_v[2],
    const float last_v_ab_v[2],
    const float i_ab_a[2],
    float vbus_v
) {
    float v_dq_v[2], current_setpoint, speed_setpoint, readings[2], hfi_v[2],
          v_ab_v[2];
    struct motor_state_t motor_state;
    enum controller_mode_t controller_mode;

    controller_mode = g_controller_state.mode;

    /* Update the state estimate with those values */
    g_estimator.update_state_estimate(
        i_ab_a, last_v_ab_v,
        g_controller_state.speed_setpoint > 0.0f ? 1.0 : -1.0f);
    g_estimator.get_state_estimate(motor_state);

    /*
    Handle stop condition -- set Vd, Vq to zero and hope the resulting hard
    stop doesn't break anything. Trigger at current limit.
    */
    if (i_ab_a[0] * i_ab_a[0] + i_ab_a[1] * i_ab_a[1] >
            HARD_CURRENT_LIMIT_A * HARD_CURRENT_LIMIT_A ||
            g_controller_state.fault) {
        g_controller_state.fault = true;
        g_controller_state.mode = controller_mode = CONTROLLER_STOPPED;
    }

    if (controller_mode == CONTROLLER_STOPPED) {
        out_v_ab_v[0] = out_v_ab_v[1] = v_dq_v[0] = v_dq_v[1] = 0.0f;
        g_estimator.reset_state();
        g_current_controller.reset_state();
        g_speed_controller.reset_state();
        g_alignment_controller.reset_state();
        readings[0] = readings[1] = 0.0f;
    } else if (g_estimator.is_converged()) {
        /* Update the speed controller setpoint */
        if (controller_mode == CONTROLLER_TORQUE ||
                controller_mode == CONTROLLER_STOPPING) {
            speed_setpoint = motor_state.angular_velocity_rad_per_s;
            g_controller_state.speed_setpoint = speed_setpoint;
        } else {
            speed_setpoint = g_controller_state.speed_setpoint;
        }
        g_speed_controller.set_setpoint(speed_setpoint);

        /*
        Update the current controller with the output of the speed controller
        when in speed control mode.
        */
        current_setpoint = g_speed_controller.update(motor_state);
        if (controller_mode == CONTROLLER_TORQUE ||
                controller_mode == CONTROLLER_STOPPING) {
            current_setpoint = g_controller_state.current_setpoint;
        } else {
            g_controller_state.current_setpoint = current_setpoint;
        }

        /* Update the stator current controller setpoint. */
        g_current_controller.set_setpoint(current_setpoint);

        /* Run the current controller and obtain new Vd and Vq outputs. */
        g_current_controller.update(v_dq_v,
                                    motor_state.i_dq_a,
                                    motor_state.angular_velocity_rad_per_s,
                                    vbus_v);

        /* Add high-frequency injection voltage when required. */
        g_estimator.get_hfi_carrier_dq_v(hfi_v);
        g_estimator.get_hfi_readings(readings);
        v_dq_v[0] += hfi_v[0];
        v_dq_v[1] += hfi_v[1];

        /*
        Transform Vd and Vq into the alpha-beta frame, and set the PWM output
        accordingly.
        */
        g_estimator.get_est_v_alpha_beta_from_v_dq(out_v_ab_v, v_dq_v);
    } else {
        /*
        Estimator is still aligning/converging; run the alignment controller
        and add the HFI voltages on top.
        */
        g_alignment_controller.update(v_ab_v, i_ab_a, motor_state.angle_rad,
                                      vbus_v);
        g_estimator.get_hfi_carrier_dq_v(hfi_v);
        g_estimator.get_hfi_readings(readings);

        /*
        Transform Vd and Vq into the alpha-beta frame, and set the PWM output
        accordingly.
        */
        g_estimator.get_est_v_alpha_beta_from_v_dq(out_v_ab_v, hfi_v);
        out_v_ab_v[0] += v_ab_v[0];
        out_v_ab_v[1] += v_ab_v[1];

        v_dq_v[0] = v_dq_v[1] = 0.0f;
    }

    g_estimator_consistency = g_estimator.get_consistency();
    g_motor_state.angular_velocity_rad_per_s =
        motor_state.angular_velocity_rad_per_s;
    g_motor_state.angle_rad = motor_state.angle_rad;
    g_motor_state.i_dq_a[0] = motor_state.i_dq_a[0];
    g_motor_state.i_dq_a[1] = motor_state.i_dq_a[1];
    g_v_dq_v[0] = v_dq_v[0];
    g_v_dq_v[1] = v_dq_v[1];
    g_vbus_v = vbus_v;
    g_estimator_hfi[0] = readings[0];
    g_estimator_hfi[1] = readings[1];
}


void systick_cb(void) {
    struct controller_state_t state;

    state = const_cast<struct controller_state_t&>(g_controller_state);
    state.time++;

    /* Motor shutdown logic -- stop when back EMF drops below 0.05 V */
    if (std::abs(g_v_dq_v[0]) < 0.05f && std::abs(g_v_dq_v[1]) < 0.05f &&
            state.mode == CONTROLLER_STOPPING) {
        g_controller_state.mode = CONTROLLER_STOPPED;
        g_controller_state.speed_setpoint = 0.0f;
        g_controller_state.current_setpoint = 0.0f;
    } else if (state.mode == CONTROLLER_STOPPING) {
        /*
        Add a small amount of negative torque to ensure the motor actually
        shuts down.
        */
        g_controller_state.current_setpoint =
            state.speed_setpoint > 0.0f ? -0.25f : 0.25f;
    } else if ((state.mode == CONTROLLER_SPEED ||
                state.mode == CONTROLLER_TORQUE) &&
                state.time - state.last_setpoint_update > THROTTLE_TIMEOUT_MS) {
        /*
        Stop gracefully if the present command mode doesn't provide an
        updated setpoint in the required period.
        */
        g_controller_state.mode = CONTROLLER_STOPPING;
    }

    g_controller_state.time = state.time;
}


void rc_pwm_cb(uint32_t width_us, uint32_t period_us) {
    g_pwm_width_us = width_us;
    g_pwm_period_us = period_us;
    g_pwm_updated = true;
}


static void node_init(void) {
    hal_set_can_dtid_filter(
        0u, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND,
        uavcan::equipment::esc::RawCommand::DefaultDataTypeID);
    hal_set_can_dtid_filter(
        0u, UAVCAN_EQUIPMENT_ESC_RPMCOMMAND,
        uavcan::equipment::esc::RPMCommand::DefaultDataTypeID);

    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE,
        uavcan::protocol::param::ExecuteOpcode::DefaultDataTypeID);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_GETSET,
        uavcan::protocol::param::GetSet::DefaultDataTypeID);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE,
        uavcan::protocol::file::BeginFirmwareUpdate::DefaultDataTypeID);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_GETNODEINFO,
        uavcan::protocol::GetNodeInfo::DefaultDataTypeID);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_RESTARTNODE,
        uavcan::protocol::RestartNode::DefaultDataTypeID);
}


static void __attribute__((noreturn)) node_run(
    Configuration& configuration,
    const struct motor_params_t& motor_params,
    const struct pwm_params_t& pwm_params
) {
    size_t length, i;
    uint32_t message_id, current_time, width_us, period_us, valid_pwm_pulses,
             node_status_time, esc_status_time, custom_status_time,
             node_status_interval, esc_status_interval,
             custom_status_interval;
    uint8_t filter_id, custom_status_transfer_id, node_status_transfer_id,
            esc_status_transfer_id, node_id, esc_index, message[8];
    enum hal_status_t status;
    enum controller_mode_t mode;
    float setpoint, value, is_a, vs_v, v_dq_v[2];
    bool got_setpoint, param_valid, wants_bootloader_restart;
    struct param_t param;
    struct motor_state_t motor_state;
    uint16_t custom_status_dtid;

    custom_status_transfer_id = node_status_transfer_id =
        esc_status_transfer_id = 0u;

    custom_status_time = node_status_time = esc_status_time = 0u;

    esc_index = (uint8_t)
        configuration.get_param_value_by_index(PARAM_UAVCAN_ESC_INDEX);
    custom_status_dtid = (uint16_t)
        configuration.get_param_value_by_index(PARAM_CUSTOM_ESCSTATUS_ID);
    node_id = (uint8_t)(100u + esc_index); /* FIXME: load from bootloader/app common */

    UAVCANTransferManager broadcast_manager(node_id);
    UAVCANTransferManager service_manager(node_id);

    uavcan::equipment::esc::RawCommand raw_cmd;
    uavcan::equipment::esc::RPMCommand rpm_cmd;
    uavcan::protocol::param::ExecuteOpcode::Request xo_req;
    uavcan::protocol::param::GetSet::Request gs_req;
    uavcan::protocol::RestartNode::Request rn_req;

    node_status_interval = 900u;
    esc_status_interval = (uint32_t)(configuration.get_param_value_by_index(
        PARAM_UAVCAN_ESCSTATUS_INTERVAL) * 0.001f);
    custom_status_interval = (uint32_t)(configuration.get_param_value_by_index(
        PARAM_CUSTOM_ESCSTATUS_INTERVAL) * 0.001f);

    valid_pwm_pulses = 0u;

    while (true) {
        current_time = g_controller_state.time;
        got_setpoint = false;

        motor_state = const_cast<struct motor_state_t&>(g_motor_state);
        v_dq_v[0] = g_v_dq_v[0];
        v_dq_v[1] = g_v_dq_v[1];

        /*
        Check for UAVCAN commands (FIFO 0) -- these are all broadcasts
        */
        status = hal_receive_can_message(0u, &filter_id, &message_id,
                                         &length, message);
        if (status == HAL_STATUS_OK) {
            broadcast_manager.receive_frame(current_time, message_id, length,
                                            message);
        }

        if (broadcast_manager.is_rx_done()) {
            if (filter_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND &&
                broadcast_manager.decode_esc_rawcommand(raw_cmd)) {
                got_setpoint = true;
                mode = CONTROLLER_TORQUE;
                setpoint = motor_params.max_current_a *
                           (float)raw_cmd.cmd[esc_index] * (1.0f / 8192.0f);
            } else if (filter_id == UAVCAN_EQUIPMENT_ESC_RPMCOMMAND &&
                        broadcast_manager.decode_esc_rpmcommand(rpm_cmd)) {
                got_setpoint = true;
                mode = CONTROLLER_SPEED;
                setpoint = (float)rpm_cmd.rpm[esc_index] * (1.0f / 60.0f) *
                          (float)motor_params.num_poles * (float)M_PI;
            }
        }

        /*
        Check for UAVCAN service requests (FIFO 1) -- only process if the
        first byte of the data is the local node ID
        */
        status = hal_receive_can_message(1u, &filter_id, &message_id,
                                         &length, message);
        if (status == HAL_STATUS_OK && message[0] == node_id) {
            service_manager.receive_frame(current_time, message_id, length,
                                          message);
        }

        /*
        Don't process service requests until the last service response is
        completely sent, to avoid overwriting the TX buffer.
        */
        if (service_manager.is_rx_done() && service_manager.is_tx_done()) {
            if (filter_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE &&
                    service_manager.decode_executeopcode_request(xo_req)) {
                /*
                Return OK if the opcode is understood and the controller is
                stopped, otherwise reject.
                */
                uavcan::protocol::param::ExecuteOpcode::Response xo_resp;
                xo_resp.ok = false;
                if (g_controller_state.mode == CONTROLLER_STOPPED &&
                        xo_req.opcode == xo_req.OPCODE_SAVE) {
                    configuration.write_params();
                    xo_resp.ok = true;
                } else if (g_controller_state.mode == CONTROLLER_STOPPED &&
                        xo_req.opcode == xo_req.OPCODE_ERASE) {
                    /*
                    Set all parameters to default values, then erase the flash
                    */
                    for (i = 0u; i < NUM_PARAMS; i++) {
                        configuration.get_param_by_index(param, (uint8_t)i);
                        configuration.set_param_value_by_index(
                            (uint8_t)i, param.default_value);
                    }
                    configuration.write_params();
                    xo_resp.ok = true;
                }
                service_manager.encode_executeopcode_response(xo_resp);
            } else if (filter_id == UAVCAN_PROTOCOL_PARAM_GETSET &&
                    service_manager.decode_getset_request(gs_req)) {
                uavcan::protocol::param::GetSet::Response resp;

                if (!gs_req.name.empty()) {
                    param_valid = configuration.get_param_by_name(
                        param, gs_req.name.c_str());
                } else {
                    param_valid = configuration.get_param_by_index(
                        param, gs_req.index);
                }

                if (param_valid) {
                    if (param.public_type == PARAM_TYPE_FLOAT &&
                            !gs_req.value.value_float.empty()) {
                        value = gs_req.value.value_float[0];
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    } else if (param.public_type == PARAM_TYPE_INT &&
                            !gs_req.value.value_int.empty()) {
                        value = (float)gs_req.value.value_int[0];
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    }

                    value = configuration.get_param_value_by_index(
                        param.index);

                    resp.name = gs_req.name;
                    if (param.public_type == PARAM_TYPE_FLOAT) {
                        resp.value.value_float.push_back(value);
                        resp.default_value.value_float.push_back(
                            param.default_value);
                        resp.min_value.value_float.push_back(param.min_value);
                        resp.max_value.value_float.push_back(param.max_value);
                    } else if (param.public_type == PARAM_TYPE_INT) {
                        resp.value.value_int.push_back((int64_t)value);
                        resp.default_value.value_int.push_back(
                            (int64_t)param.default_value);
                        resp.min_value.value_int.push_back(
                            (int64_t)param.min_value);
                        resp.max_value.value_int.push_back(
                            (int64_t)param.max_value);
                    }
                }

                service_manager.encode_getset_response(resp);
            } else if (filter_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE) {
                uavcan::protocol::file::BeginFirmwareUpdate::Response resp;

                /*
                Don't actually need to decode since we don't care about the
                request data
                */
                if (g_controller_state.mode == CONTROLLER_STOPPED) {
                    resp.error = resp.ERROR_OK;
                    g_controller_state.fault = true;
                    wants_bootloader_restart = true;
                } else {
                    resp.error = resp.ERROR_INVALID_MODE;
                }
                service_manager.encode_beginfirmwareupdate_response(resp);
            } else if (filter_id == UAVCAN_PROTOCOL_GETNODEINFO) {
                uavcan::protocol::GetNodeInfo::Response resp;

                /* Empty request so don't need to decode */
                resp.status.uptime_sec = current_time / 1000u;
                resp.status.status_code = resp.status.STATUS_OK;
                resp.status.vendor_specific_status_code = 0u;
                resp.software_version.major =
                    flash_app_descriptor.major_version;
                resp.software_version.minor =
                    flash_app_descriptor.minor_version;
                resp.software_version.optional_field_mask =
                    resp.software_version.OPTIONAL_FIELD_MASK_VCS_COMMIT |
                    resp.software_version.OPTIONAL_FIELD_MASK_IMAGE_CRC;
                resp.software_version.vcs_commit =
                    flash_app_descriptor.vcs_commit;
                resp.software_version.image_crc =
                    flash_app_descriptor.image_crc;
                resp.hardware_version.major = 0u;
                resp.hardware_version.minor = 0u;
                // resp.hardware_version.unique_id;
                resp.name = "com.thiemar.S2740VC";

                service_manager.encode_getnodeinfo_response(resp);
            } else if (filter_id == UAVCAN_PROTOCOL_RESTARTNODE &&
                    service_manager.decode_restartnode_request(rn_req)) {
                uavcan::protocol::RestartNode::Response resp;

                /*
                Restart if the magic number is correct and the controller is
                currently stopped, otherwise reject.
                */
                if (g_controller_state.mode == CONTROLLER_STOPPED &&
                        rn_req.magic_number == rn_req.MAGIC_NUMBER) {
                    resp.ok = true;
                    hal_restart();
                } else {
                    resp.ok = false;
                }
                service_manager.encode_restartnode_response(resp);
            }
        }

        if (broadcast_manager.is_tx_done()) {
            is_a = __VSQRTF(motor_state.i_dq_a[0] * motor_state.i_dq_a[0] +
                            motor_state.i_dq_a[1] * motor_state.i_dq_a[1]);
            vs_v = __VSQRTF(v_dq_v[0] * v_dq_v[0] + v_dq_v[1] * v_dq_v[1]);

            if (custom_status_interval && current_time - custom_status_time >=
                    custom_status_interval) {
                thiemar::equipment::esc::Status msg;

                msg.i_dq[0] = motor_state.i_dq_a[0];
                msg.i_dq[1] = motor_state.i_dq_a[1];
                msg.i_setpoint = g_controller_state.current_setpoint;
                msg.v_dq[0] = v_dq_v[0];
                msg.v_dq[1] = v_dq_v[1];
                msg.hfi_dq[0] = g_estimator_hfi[0];
                msg.hfi_dq[1] = g_estimator_hfi[1];
                msg.consistency = g_estimator_consistency;
                msg.vbus = g_vbus_v;
                msg.temperature = 273.15f + hal_get_temperature_degc();
                msg.angle = motor_state.angle_rad;
                msg.rpm = (int32_t)(g_motor_state.angular_velocity_rad_per_s *
                    60.0f / ((float)M_PI * (float)motor_params.num_poles));
                msg.rpm_setpoint = (int32_t)(g_controller_state.speed_setpoint *
                    60.0f / ((float)M_PI * (float)motor_params.num_poles));
                msg.esc_index = esc_index;
                broadcast_manager.encode_custom_status(
                    custom_status_transfer_id++, custom_status_dtid, msg);
                custom_status_time = current_time;
            } else if (esc_status_interval &&
                    current_time - esc_status_time >= esc_status_interval) {
                uavcan::equipment::esc::Status msg;

                msg.voltage = g_vbus_v;
                msg.current = is_a * vs_v / g_vbus_v;
                msg.temperature = 273.15f + hal_get_temperature_degc();
                msg.rpm = (int32_t)(g_motor_state.angular_velocity_rad_per_s *
                    60.0f / ((float)M_PI * (float)motor_params.num_poles));
                msg.power_rating_pct = (uint8_t)(100.0f * (is_a * vs_v) /
                    (motor_params.max_current_a * motor_params.max_voltage_v));
                msg.esc_index = esc_index;
                broadcast_manager.encode_esc_status(
                    esc_status_transfer_id++, msg);
                esc_status_time = current_time;
            } else if (current_time - node_status_time >=
                        node_status_interval) {
                uavcan::protocol::NodeStatus msg;

                msg.uptime_sec = current_time / 1000u;
                msg.status_code = msg.STATUS_OK;
                msg.vendor_specific_status_code = 0u;
                broadcast_manager.encode_nodestatus(
                    node_status_transfer_id++, msg);
                node_status_time = current_time;
            }
        }

        /* Transmit CAN frames if available */
        if (broadcast_manager.transmit_frame(message_id, length, message)) {
            (void)hal_transmit_can_message(0u, message_id, length, message);
        }

        if (service_manager.transmit_frame(message_id, length, message)) {
            (void)hal_transmit_can_message(1u, message_id, length, message);
        }

        /* Check for PWM input */
        if (g_pwm_updated) {
            width_us = g_pwm_width_us;
            period_us = g_pwm_period_us;

            /* PWM arm/disarm logic */
            if (width_us > RC_PWM_MIN_WIDTH_US &&
                    width_us < RC_PWM_MAX_WIDTH_US &&
                    period_us > RC_PWM_MIN_PERIOD_US &&
                    period_us < RC_PWM_MAX_PERIOD_US) {

                if (valid_pwm_pulses >= RC_PWM_ARM_PULSES &&
                        width_us > pwm_params.throttle_pulse_min_us) {
                    got_setpoint = true;
                    mode = pwm_params.use_speed_controller ?
                            CONTROLLER_SPEED : CONTROLLER_TORQUE;
                    setpoint = pwm_calculate_setpoint(pwm_params, width_us);
                } else {
                    valid_pwm_pulses++;
                }

                /* Disable CAN if we have valid PWM */
                hal_disable_can_transmit();
            } else if (valid_pwm_pulses > 0u) {
                valid_pwm_pulses--;
            }

            g_pwm_updated = false;
        }

        /* Update the controller mode and setpoint */
        if (!g_controller_state.fault && got_setpoint) {
            if (mode == CONTROLLER_SPEED) {
                g_controller_state.speed_setpoint = setpoint;
                g_controller_state.mode = mode;
                g_controller_state.last_setpoint_update = current_time;
            } else if (mode == CONTROLLER_TORQUE) {
                g_controller_state.current_setpoint = setpoint;
                g_controller_state.mode = mode;
                g_controller_state.last_setpoint_update = current_time;
            }
        }

        /*
        Only restart into the bootloader if the acknowledgement message has
        been sent, and we're otherwise unoccupied.
        */
        if (g_controller_state.mode == CONTROLLER_STOPPED &&
                broadcast_manager.is_tx_done() && wants_bootloader_restart) {
            /* TODO */
            hal_restart();
        }
    }
}


static float pwm_calculate_setpoint(
    const struct pwm_params_t &params,
    uint32_t width_us
) {
    float setpoint, scale_factor, sign;

    /* Constrain the PWM signal to full scale */
    width_us = std::min((uint32_t)params.throttle_pulse_max_us,
                        (uint32_t)width_us);

    scale_factor = 1.0f / (float)(params.throttle_pulse_max_us -
                                  params.throttle_pulse_min_us);
    setpoint = (float)(width_us - params.throttle_pulse_min_us) *
               scale_factor;
    setpoint -= params.control_offset;
    if (std::abs(setpoint) <=
            (float)params.throttle_deadband_us * scale_factor) {
        setpoint = 0.0f;
        sign = 1.0f;
    } else if (setpoint < 0.0f) {
        sign = -1.0f;
    } else {
        sign = 1.0f;
    }

    switch (params.control_curve) {
        case pwm_params_t::SQRT:
            setpoint = __VSQRTF(std::abs(setpoint));
            break;
        case pwm_params_t::QUADRATIC:
            setpoint *= setpoint;
            break;
        default:
            break;
    }
    setpoint = params.control_min +
               setpoint * (params.control_max - params.control_min);

    return setpoint * sign;
}


int __attribute__((externally_visible,noreturn)) main(void) {
    Configuration configuration;
    struct control_params_t control_params;
    struct motor_params_t motor_params;
    struct pwm_params_t pwm_params;
    uint32_t i;
    float i_samples[4], v_samples[4], rs_r, ls_h, limit_speed;

    hal_reset();
    hal_set_pwm_state(HAL_PWM_STATE_LOW);

    /* Read parameters from flash */
    configuration.read_motor_params(motor_params);
    configuration.read_control_params(control_params);
    configuration.read_pwm_params(pwm_params);

    /* Estimate motor parameters */
    g_parameter_estimator_done = false;
    g_parameter_estimator.start_estimation(hal_control_t_s);
    hal_set_high_frequency_callback(identification_cb);

    hal_set_pwm_state(HAL_PWM_STATE_RUNNING);
    g_controller_state.mode = CONTROLLER_IDENTIFYING;

    while (!g_parameter_estimator_done &&
            g_controller_state.mode == CONTROLLER_IDENTIFYING);

    hal_set_pwm_state(HAL_PWM_STATE_LOW);
    g_controller_state.mode = CONTROLLER_STOPPED;

    hal_set_high_frequency_callback(NULL);

    /*
    If this fails, there was an overcurrent condition during Rs/Ls testing,
    which is really bad.
    */
    esc_assert(g_parameter_estimator_done);

    /* Calculate Rs and Ls based on the measurements */
    for (i = 0; i < 4; i++) {
        i_samples[i] = g_i_samples[i];
        v_samples[i] = g_v_samples[i];
    }

    ParameterEstimator::calculate_r_l_from_samples(
        rs_r, ls_h, v_samples, i_samples);

    if (!std::isnan(rs_r) && 1e-3f < rs_r && rs_r < 1.0f) {
        motor_params.rs_r = rs_r;
    }
    if (!std::isnan(ls_h) && 5e-6f < ls_h && ls_h < 1e-2f) {
        motor_params.ls_h = ls_h;
    }

    /*
    This is not strictly necessary as the values are never written to
    flash, but it does enable the measurements to be read via the
    parameter interfaces.
    */
    configuration.set_param_value_by_index(PARAM_MOTOR_RS, rs_r);
    configuration.set_param_value_by_index(PARAM_MOTOR_LS, ls_h);

    /* Initialize the system with the motor parameters */
    g_estimator.set_params(motor_params, control_params,
                           hal_control_t_s);
    g_current_controller.set_params(motor_params, control_params,
                                    hal_control_t_s);
    g_speed_controller.set_params(motor_params, control_params,
                                  hal_control_t_s);
    g_alignment_controller.set_params(motor_params, control_params,
                                      hal_control_t_s);

    /*
    Calculate the motor's maximum (electrical) speed based on Vbus at startup
    time, the maximum modulation limit, and the motor's flux linkage.
    */
    limit_speed = std::min(0.95f * g_vbus_v, motor_params.max_voltage_v) /
                  motor_params.phi_v_s_per_rad;
    if (motor_params.max_speed_rad_per_s > limit_speed) {
        motor_params.max_speed_rad_per_s = limit_speed;
    }

    /* Start PWM */
    hal_set_pwm_state(HAL_PWM_STATE_RUNNING);

    /*
    After starting the ISR tasks we are no longer able to access g_estimator,
    g_current_controller and g_speed_controller, since they're updated from
    the ISRs and not declared volatile.
    */
    hal_set_high_frequency_callback(control_cb);
    hal_set_low_frequency_callback(systick_cb);
    hal_set_rc_pwm_callback(rc_pwm_cb);

    node_init();

    node_run(configuration, motor_params, pwm_params);
}
