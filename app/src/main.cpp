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
#include <cmath>
#include <algorithm>
#include "hal.h"
#include "can.h"
#include "park.h"
#include "perf.h"
#include "estimator.h"
#include "controller.h"
#include "configuration.h"
#include "uavcan.h"


#include "uavcan/protocol/param/ExecuteOpcode.hpp"
#include "uavcan/protocol/param/GetSet.hpp"
#include "uavcan/protocol/file/BeginFirmwareUpdate.hpp"
#include "uavcan/protocol/GetNodeInfo.hpp"
#include "uavcan/protocol/NodeStatus.hpp"
#include "uavcan/protocol/RestartNode.hpp"
#include "uavcan/protocol/enumeration/Begin.hpp"
#include "uavcan/protocol/enumeration/Indication.hpp"
#include "uavcan/equipment/esc/RawCommand.hpp"
#include "uavcan/equipment/esc/RPMCommand.hpp"
#include "uavcan/equipment/esc/Status.hpp"
#include "thiemar/equipment/esc/Status.hpp"
#include "uavcan/equipment/indication/BeepCommand.hpp"


enum uavcan_dtid_filter_id_t {
    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE = 0u,
    UAVCAN_PROTOCOL_PARAM_GETSET,
    UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE,
    UAVCAN_PROTOCOL_GETNODEINFO,
    UAVCAN_PROTOCOL_RESTARTNODE,
    UAVCAN_PROTOCOL_ENUMERATION_BEGIN,
    UAVCAN_EQUIPMENT_ESC_RAWCOMMAND,
    UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND
};

enum controller_mode_t {
    CONTROLLER_STOPPED,
    CONTROLLER_IDENTIFYING,
    CONTROLLER_VOLTAGE,
    CONTROLLER_STOPPING,
    CONTROLLER_IDLING,
    CONTROLLER_ABORTING
};

struct controller_state_t {
    uint32_t time;

    float voltage_setpoint;
    float internal_speed_setpoint;
    float current_setpoint;

    enum controller_mode_t mode;
    bool fault;
};

struct audio_state_t {
    uint32_t off_time;
    float phase_rad;
    float angular_velocity_rad_per_u;
    float volume_v;
};

struct controller_constants_t {
    float accel_current_a;
    float max_current_a;
    float max_voltage_v;
    float max_vbus_v;
    float braking_current_a;
    float effective_inv_r;
    float control_lpf_coeff;
};


#define ENUMERATION_VELOCITY_THRESHOLD_RAD_PER_S float(2.0 * M_PI * 2.0)


/*
"Owned" by identification_cb / control_cb (only one will be running at a given
stage in execution).
*/
static StateEstimator g_estimator;
static ParameterEstimator g_parameter_estimator;
static DQCurrentController g_current_controller;
static struct controller_constants_t g_controller_constants;


/*
Globally-visible board state updated by high-frequency callbacks and read by
the low-frequency callback (doesn't matter if it's slightly out of sync as
it's only used for reporting).
*/
volatile static struct controller_state_t g_controller_state;
volatile static struct motor_state_t g_motor_state;
volatile static struct audio_state_t g_audio_state;


/* Bus and output voltage for reporting purposes */
volatile static float g_vbus_v;
volatile static float g_phi_v_s_per_rad;


/* Motor parameter estimation state -- only used on startup */
volatile static float g_measured_rs_r;
volatile static float g_measured_ls_h;


/* Enumeration state */
volatile static float g_enumeration_velocity_rad_per_s;
volatile static bool g_enumeration_active;


/* Written to the firmware image in post-processing */
extern volatile struct bootloader_app_descriptor flash_app_descriptor;


/*
Throttle timeout -- if we don't receive a setpoint update in this long,
we spin the motor down
*/
#define THROTTLE_TIMEOUT_MS 500u


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


static void node_init(uint8_t node_id, Configuration& configuration);
static void node_run(
    uint8_t node_id,
    Configuration& configuration,
    const struct motor_params_t& motor_params,
    const struct control_params_t& control_params
);


void identification_cb(
    float out_v_ab_v[2],
    const float last_v_ab_v[2],
    const float i_ab_a[2],
    float vbus_v
) {
    float r, l;

    /* Handle stop condition -- set Vd, Vq to zero. */
    if (g_controller_state.fault) {
        g_controller_state.fault = true;
        g_controller_state.mode = CONTROLLER_STOPPED;
    }

    if (g_controller_state.mode == CONTROLLER_STOPPED) {
        out_v_ab_v[0] = out_v_ab_v[1] = 0.0f;
    } else {
        /* Update the parameter estimate with those values */
        g_parameter_estimator.update_parameter_estimate(i_ab_a, last_v_ab_v);
        g_parameter_estimator.get_v_alpha_beta_v(out_v_ab_v);

        if (g_parameter_estimator.is_estimation_complete()) {
            g_parameter_estimator.calculate_r_l(r, l);
            g_measured_rs_r = r;
            g_measured_ls_h = l;
            g_controller_state.mode = CONTROLLER_STOPPED;
        }
    }

    g_vbus_v = vbus_v;
}


void control_cb(
    float out_v_ab_v[2],
    const float last_v_ab_v[2],
    const float i_ab_a[2],
    float vbus_v
) {
    float v_dq_v[2], phase, audio_v, voltage_setpoint, current_setpoint,
          internal_speed_setpoint, phi, temp, current_limit, accel_limit;
    struct motor_state_t motor_state;
    enum controller_mode_t mode;

    voltage_setpoint = g_controller_state.voltage_setpoint;
    internal_speed_setpoint = g_controller_state.internal_speed_setpoint;
    current_setpoint = g_controller_state.current_setpoint;

    mode = g_controller_state.mode;

    audio_v = 0.0f;

    /* Update the state estimate with the latest measurements */
    g_estimator.update_state_estimate(i_ab_a, last_v_ab_v,
                                      internal_speed_setpoint);
    g_estimator.get_state_estimate(motor_state);

    /* Get the latest phi estimate */
    phi = g_estimator.get_phi_estimate();

    if (g_controller_state.fault) {
        g_estimator.reset_state();
        g_controller_state.fault = true;
        g_controller_state.mode = CONTROLLER_STOPPED;
    } else if (g_audio_state.off_time) {
        /* Calculate audio component */
        audio_v = g_audio_state.volume_v;
        phase = g_audio_state.phase_rad;
        phase += g_audio_state.angular_velocity_rad_per_u;

        if (phase > float(M_PI)) {
            phase -= float(2.0 * M_PI);
        }
        if (phase < 0.0f) {
            audio_v = -audio_v;
        }

        g_audio_state.phase_rad = phase;
        g_audio_state.off_time--;
    }

    /*
    Outer control loop -- speed or aerodynamic torque, plus spin-up and
    spin-down routines.
    */
    if (mode == CONTROLLER_STOPPING || mode == CONTROLLER_ABORTING) {
        /*
        When stopping, the speed controller setpoint tracks the current
        angular velocity.
        */
        internal_speed_setpoint = motor_state.angular_velocity_rad_per_s;
        current_setpoint = std::min(
            -(1.0f / 8.0f) * g_controller_constants.accel_current_a, -0.5f);
        if (internal_speed_setpoint < 0.0f) {
            current_setpoint = -current_setpoint;
        }

        /* Stop when back EMF drops below 0.25 V */
        if (motor_state.v_dq_v[0] * motor_state.v_dq_v[0] +
                motor_state.v_dq_v[1] * motor_state.v_dq_v[1] < 0.25f * 0.25f) {
            g_controller_state.mode = mode = CONTROLLER_STOPPED;
        }
    } else if (mode == CONTROLLER_IDLING) {
        /* Rotate at the idle speed */
        internal_speed_setpoint = 2.0f * M_PI;
        current_setpoint = g_controller_constants.accel_current_a;
        if (voltage_setpoint < 0.0f) {
            current_setpoint = -current_setpoint;
        }
    } else if (mode == CONTROLLER_VOLTAGE) {
        internal_speed_setpoint =
            std::max(float(2.0 * M_PI), motor_state.angular_velocity_rad_per_s);

        temp = g_controller_constants.effective_inv_r *
               (voltage_setpoint - motor_state.v_dq_v[1]);

        /*
        If estimated back EMF is < 1.0 V (start-up or shutdown), limit
        total current to the acceleration limit
        */
        accel_limit = g_controller_constants.accel_current_a;
        if (std::abs(internal_speed_setpoint * phi) > 1.0) {
            accel_limit += current_setpoint;
        }

        if (temp > accel_limit) {
            temp = accel_limit;
        }
        if (temp < -accel_limit) {
            temp = -accel_limit;
        }

        current_setpoint += (temp - current_setpoint) *
                            g_controller_constants.control_lpf_coeff;

        current_limit = g_controller_constants.max_current_a;
        if (current_setpoint * voltage_setpoint < 0.0f) {
            /*
            If current setpoint and voltage setpoint have opposite signs,
            the motor is braking. We have a separate torque limit for that,
            so users with screw-on props can avoid unscrewing them in flight.
            */
            current_limit = g_controller_constants.braking_current_a;

            /*
            While braking, we don't want to exceed the bus voltage limit,
            which is possible with some sources as regenerative braking can
            cause significant current flow back to the source.

            If we see the bus voltage exceeding the limit, we reduce the
            braking current accordingly; the allowable braking current drops
            to zero at 10% above the maximum voltage.
            */
            if (vbus_v > g_controller_constants.max_vbus_v) {
                temp = (vbus_v - g_controller_constants.max_vbus_v) /
                       g_controller_constants.max_vbus_v;
                current_limit *=
                    std::max(0.0f, std::min(1.0f, 1.0f - 10.0f * temp));
            }
        }

        if (current_setpoint > current_limit) {
            current_setpoint = current_limit;
        }
        if (current_setpoint < -current_limit) {
            current_setpoint = -current_limit;
        }
    } else /* Normally: if (mode == CONTROLLER_STOPPED), but catch-all */ {
        internal_speed_setpoint = 0.0f;
        current_setpoint = 0.0f;
        g_current_controller.reset_state();
    }

    /*
    Inner control loop -- adjust motor voltage to maintain the current
    setpoint
    */

    /* Update the stator current controller setpoint. */
    g_current_controller.set_setpoint(current_setpoint);

    /* Run the current controller and obtain new Vd and Vq outputs. */
    g_current_controller.update(v_dq_v,
                                motor_state.i_dq_a,
                                motor_state.angular_velocity_rad_per_s,
                                vbus_v,
                                audio_v);

    /* Suppress current controller output when stopped */
    if (mode == CONTROLLER_STOPPED) {
        v_dq_v[0] = 0.0f;
        v_dq_v[1] = audio_v;
    }

    /*
    Transform Vd and Vq into the alpha-beta frame, and set the PWM output
    accordingly.
    */
    g_estimator.get_est_v_alpha_beta_from_v_dq(out_v_ab_v, v_dq_v);

    /*
    Could look at low-passing these, based on the FOC status output rate
    */
    g_controller_state.internal_speed_setpoint = internal_speed_setpoint;
    g_controller_state.current_setpoint = current_setpoint;

    g_motor_state.angular_velocity_rad_per_s =
        motor_state.angular_velocity_rad_per_s;
    g_motor_state.angle_rad = motor_state.angle_rad;
    g_motor_state.i_dq_a[0] = motor_state.i_dq_a[0];
    g_motor_state.i_dq_a[1] = motor_state.i_dq_a[1];
    g_motor_state.v_dq_v[0] = motor_state.v_dq_v[0];
    g_motor_state.v_dq_v[1] = motor_state.v_dq_v[1];
    g_vbus_v = vbus_v;
    g_phi_v_s_per_rad = phi;
}


void systick_cb(void) {
    g_controller_state.time++;

    /* Update the enumeration velocity */
    if (g_enumeration_active) {
        g_enumeration_velocity_rad_per_s +=
            (g_motor_state.angular_velocity_rad_per_s -
                g_enumeration_velocity_rad_per_s) * 0.0004f;
    } else {
        g_enumeration_velocity_rad_per_s = 0.0f;
    }
}


static void node_init(uint8_t node_id, Configuration& configuration) {
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE, true,
        uavcan::protocol::param::ExecuteOpcode::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_GETSET, true,
        uavcan::protocol::param::GetSet::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE, true,
        uavcan::protocol::file::BeginFirmwareUpdate::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_GETNODEINFO, true,
        uavcan::protocol::GetNodeInfo::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_RESTARTNODE, true,
        uavcan::protocol::RestartNode::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_ENUMERATION_BEGIN, true,
        uavcan::protocol::enumeration::Begin::DefaultDataTypeID,
        node_id);

    can_set_dtid_filter(
        0u, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND, false,
        uavcan::equipment::esc::RawCommand::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        0u, UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND, false,
        uavcan::equipment::indication::BeepCommand::DefaultDataTypeID,
        node_id);
}


static void __attribute__((noreturn)) node_run(
    uint8_t node_id,
    Configuration& configuration,
    const struct motor_params_t& motor_params,
    const struct control_params_t& control_params
) {
    size_t length;
    uint32_t message_id, current_time, node_status_time, esc_status_time,
             foc_status_time, node_status_interval, esc_status_interval,
             foc_status_interval, last_setpoint_update, enumeration_deadline;
    uint8_t filter_id, foc_status_transfer_id, node_status_transfer_id,
            esc_status_transfer_id, enumeration_indication_transfer_id,
            esc_index, message[8], broadcast_filter_id, service_filter_id;
    enum controller_mode_t mode;
    float setpoint, value, power_w, to_rpm;
    bool got_setpoint, param_valid, wants_bootloader_restart;
    struct param_t param;
    struct motor_state_t motor_state;
    uint16_t foc_status_dtid;

    UAVCANTransferManager broadcast_manager(node_id);
    UAVCANTransferManager service_manager(node_id);

    uavcan::equipment::esc::RawCommand raw_cmd;
    uavcan::equipment::indication::BeepCommand beep_cmd;
    uavcan::protocol::param::ExecuteOpcode::Request xo_req;
    uavcan::protocol::param::GetSet::Request gs_req;
    uavcan::protocol::RestartNode::Request rn_req;
    uavcan::protocol::enumeration::Begin::Request enum_req;

    foc_status_transfer_id = node_status_transfer_id =
        esc_status_transfer_id = enumeration_indication_transfer_id = 0u;

    broadcast_filter_id = service_filter_id = 0xFFu;

    foc_status_time = node_status_time = esc_status_time =
        last_setpoint_update = enumeration_deadline = 0u;

    wants_bootloader_restart = false;

    node_status_interval = 900u;

    to_rpm = float(60.0 / M_PI) / float(motor_params.num_poles);

    while (true) {
        current_time = g_controller_state.time;
        got_setpoint = false;
        setpoint = 0.0f;

        /* Allow UAVCAN parameters to be changed without restarting */
        esc_index = (uint8_t)
            configuration.get_param_value_by_index(PARAM_UAVCAN_ESC_INDEX);
        foc_status_dtid = (uint16_t)
            configuration.get_param_value_by_index(PARAM_THIEMAR_STATUS_ID);
        esc_status_interval = (uint32_t)(
            configuration.get_param_value_by_index(
            PARAM_UAVCAN_ESCSTATUS_INTERVAL) * 0.001f);
        foc_status_interval = (uint32_t)(
            configuration.get_param_value_by_index(
            PARAM_THIEMAR_STATUS_INTERVAL) * 0.001f);

        /*
        Check for UAVCAN commands (FIFO 0) -- these are all broadcasts
        */
        while (can_rx(0u, &filter_id, &message_id, &length, message)) {
            uavcan::TransferCRC crc;
            /* Filter IDs are per-FIFO, so convert this back to the bank index */
            filter_id = (uint8_t)(filter_id + UAVCAN_EQUIPMENT_ESC_RAWCOMMAND);

            switch (filter_id) {
                case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND:
                    crc = uavcan::equipment::esc::RawCommand::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND:
                    crc = uavcan::equipment::indication::BeepCommand::getDataTypeSignature().toTransferCRC();
                    break;
                default:
                    break;
            }
            broadcast_manager.receive_frame(current_time, message_id, crc,
                                            length, message);

            if (broadcast_manager.is_rx_done()) {
                broadcast_filter_id = filter_id;
                break;
            }
        }

        if (broadcast_manager.is_rx_done()) {
            if (broadcast_filter_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND &&
                    broadcast_manager.decode(raw_cmd) &&
                    esc_index < raw_cmd.cmd.size() &&
                    raw_cmd.cmd[esc_index] != 0) {
                got_setpoint = true;
                mode = CONTROLLER_VOLTAGE;
                /* Scale 0-8191 to represent 0 to maximum voltage. */
                value = g_vbus_v;
                setpoint = std::min(motor_params.max_voltage_v, value) *
                           float(raw_cmd.cmd[esc_index]) *
                           float(1.0 / 8192.0);
                if (raw_cmd.cmd[esc_index] > 0 &&
                        setpoint < motor_params.accel_voltage_v) {
                    setpoint = motor_params.accel_voltage_v;
                }
                if (raw_cmd.cmd[esc_index] < 0 &&
                        setpoint > -motor_params.accel_voltage_v) {
                    setpoint = -motor_params.accel_voltage_v;
                }
            } else if (broadcast_filter_id ==
                            UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND &&
                       broadcast_manager.decode(beep_cmd)) {
                /* Set up audio generation */
                g_audio_state.off_time = uint32_t(beep_cmd.duration *
                                                  (1.0f / hal_control_t_s));
                g_audio_state.angular_velocity_rad_per_u =
                    float(2.0 * M_PI) * hal_control_t_s *
                    std::max(100.0f, beep_cmd.frequency);
                g_audio_state.volume_v = motor_params.accel_voltage_v;
            }

            broadcast_manager.receive_acknowledge();
        }

        /*
        Check for UAVCAN service requests (FIFO 1) -- only process if the
        first byte of the data is the local node ID
        */
        while (can_rx(1u, &filter_id, &message_id, &length, message)) {
            uavcan::TransferCRC crc;
            switch (filter_id) {
                case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE:
                    crc = uavcan::protocol::param::ExecuteOpcode::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_PARAM_GETSET:
                    crc = uavcan::protocol::param::GetSet::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE:
                    crc = uavcan::protocol::file::BeginFirmwareUpdate::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_GETNODEINFO:
                    crc = uavcan::protocol::GetNodeInfo::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_RESTARTNODE:
                    crc = uavcan::protocol::RestartNode::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_ENUMERATION_BEGIN:
                    crc = uavcan::protocol::enumeration::Begin::getDataTypeSignature().toTransferCRC();
                    break;
                default:
                    break;
            }
            service_manager.receive_frame(current_time, message_id, crc,
                                          length, message);

            if (service_manager.is_rx_done()) {
                service_filter_id = filter_id;
                break;
            }
        }

        /*
        Don't process service requests until the last service response is
        completely sent, to avoid overwriting the TX buffer.
        */
        if (service_manager.is_rx_done() && service_manager.is_tx_done()) {
            if (service_filter_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE &&
                    service_manager.decode(xo_req)) {
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
                    configuration.reset_params();
                    configuration.write_params();
                    xo_resp.ok = true;
                }
                service_manager.encode_response<uavcan::protocol::param::ExecuteOpcode>(xo_resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_PARAM_GETSET &&
                       service_manager.decode(gs_req)) {
                uavcan::protocol::param::GetSet::Response resp;

                if (!gs_req.name.empty()) {
                    param_valid = configuration.get_param_by_name(
                        param, gs_req.name.c_str());
                } else {
                    param_valid = configuration.get_param_by_index(
                        param, (uint8_t)gs_req.index);
                }

                if (param_valid) {
                    if (param.public_type == PARAM_TYPE_FLOAT && !gs_req.name.empty() &&
                            gs_req.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
                        value = gs_req.value.to<uavcan::protocol::param::Value::Tag::real_value>();
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    } else if (param.public_type == PARAM_TYPE_INT && !gs_req.name.empty() &&
                            gs_req.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
                        value = (float)((int32_t)gs_req.value.to<uavcan::protocol::param::Value::Tag::integer_value>());
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    }

                    value = configuration.get_param_value_by_index(
                        param.index);

                    resp.name = (const char*)param.name;
                    if (param.public_type == PARAM_TYPE_FLOAT) {
                        resp.value.to<uavcan::protocol::param::Value::Tag::real_value>() = value;
                        resp.default_value.to<uavcan::protocol::param::Value::Tag::real_value>() = param.default_value;
                        resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = param.min_value;
                        resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = param.max_value;
                    } else if (param.public_type == PARAM_TYPE_INT) {
                        resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int32_t)value;
                        resp.default_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int32_t)param.default_value;
                        resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int32_t)param.min_value;
                        resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int32_t)param.max_value;
                    }
                }

                service_manager.encode_response<uavcan::protocol::param::GetSet>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE) {
                uavcan::protocol::file::BeginFirmwareUpdate::Response resp;

                /*
                Don't actually need to decode since we don't care about the
                request data
                */
                if (g_controller_state.mode == CONTROLLER_STOPPED) {
                    resp.error = resp.ERROR_OK;
                    wants_bootloader_restart = true;
                } else {
                    resp.error = resp.ERROR_INVALID_MODE;
                }
                service_manager.encode_response<uavcan::protocol::file::BeginFirmwareUpdate>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_GETNODEINFO) {
                uavcan::protocol::GetNodeInfo::Response resp;

                /* Empty request so don't need to decode */
                resp.status.uptime_sec = current_time / 1000u;
                resp.status.health = g_controller_state.fault ?
                    resp.status.HEALTH_CRITICAL :
                    resp.status.HEALTH_OK;
                resp.status.mode = resp.status.MODE_OPERATIONAL;
                resp.status.sub_mode = 0u;
                resp.status.vendor_specific_status_code =
                    (uint16_t)g_controller_state.mode;
                resp.software_version.major =
                    flash_app_descriptor.major_version;
                resp.software_version.minor =
                    flash_app_descriptor.minor_version;
                resp.software_version.optional_field_flags =
                    resp.software_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT |
                    resp.software_version.OPTIONAL_FIELD_FLAG_IMAGE_CRC;
                resp.software_version.vcs_commit =
                    flash_app_descriptor.vcs_commit;
                resp.software_version.image_crc =
                    flash_app_descriptor.image_crc;
                resp.hardware_version.major = HW_VERSION_MAJOR;
                resp.hardware_version.minor = HW_VERSION_MINOR;
                /* Set the unique ID */
                memset(resp.hardware_version.unique_id.begin(), 0u,
                       resp.hardware_version.unique_id.size());
                memcpy(resp.hardware_version.unique_id.begin(),
                       (uint8_t*)0x1ffff7ac, 12u);
                /* Set the hardware name */
                resp.name = HW_UAVCAN_NAME;

                service_manager.encode_response<uavcan::protocol::GetNodeInfo>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_RESTARTNODE &&
                    service_manager.decode(rn_req)) {
                uavcan::protocol::RestartNode::Response resp;

                /*
                Restart if the magic number is correct and the controller is
                currently stopped, otherwise reject.
                */
                if (g_controller_state.mode == CONTROLLER_STOPPED &&
                        rn_req.magic_number == rn_req.MAGIC_NUMBER) {
                    resp.ok = true;
                    wants_bootloader_restart = true;
                } else {
                    resp.ok = false;
                }
                service_manager.encode_response<uavcan::protocol::RestartNode>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_ENUMERATION_BEGIN &&
                    service_manager.decode(enum_req)) {
                uavcan::protocol::enumeration::Begin::Response resp;

                /*
                We only support enumeration for the esc_index property, and
                only while the controller is stopped.
                */
                if (g_controller_state.mode != CONTROLLER_STOPPED) {
                    resp.error = resp.ERROR_INVALID_MODE;
                } else if (enum_req.parameter_name != "esc_index") {
                    resp.error = resp.ERROR_INVALID_PARAMETER;
                } else {
                    /* All fine, start enumeration */
                    resp.error = resp.ERROR_OK;
                    enumeration_deadline =
                        current_time + 1000 * enum_req.timeout_sec;
                    g_enumeration_active = true;
                }
                service_manager.encode_response<uavcan::protocol::enumeration::Begin>(resp);
            }

            service_manager.receive_acknowledge();
        }

        /* Transmit service responses if available */
        if (can_is_ready(1u) &&
                service_manager.transmit_frame(message_id, length, message)) {
            can_tx(1u, message_id, length, message);
        }

        if (broadcast_manager.is_tx_done() && service_manager.is_tx_done() &&
                !service_manager.is_rx_in_progress(current_time)) {
            motor_state = const_cast<struct motor_state_t&>(g_motor_state);

            if (foc_status_interval && current_time - foc_status_time >=
                    foc_status_interval) {
                thiemar::equipment::esc::Status msg;

                msg.i_dq[0] = motor_state.i_dq_a[0];
                msg.i_dq[1] = motor_state.i_dq_a[1];
                msg.i_setpoint = g_controller_state.current_setpoint;

                msg.v_dq[0] = motor_state.v_dq_v[0];
                msg.v_dq[1] = motor_state.v_dq_v[1];

                msg.angle = motor_state.angle_rad;

                msg.esc_index = esc_index;

                broadcast_manager.encode_message(
                    foc_status_transfer_id++, foc_status_dtid, msg);
                foc_status_time = current_time;
            } else if (esc_status_interval &&
                    current_time - esc_status_time >= esc_status_interval) {
                uavcan::equipment::esc::Status msg;

                power_w = __VSQRTF(
                    (motor_state.i_dq_a[0] * motor_state.i_dq_a[0] +
                     motor_state.i_dq_a[1] * motor_state.i_dq_a[1]) *
                    (motor_state.v_dq_v[0] * motor_state.v_dq_v[0] +
                     motor_state.v_dq_v[1] * motor_state.v_dq_v[1])
                );

                msg.voltage = g_vbus_v;
                msg.current = power_w / g_vbus_v;
                /*
                If Q current has opposite sign to Q voltage, the flow is
                reversed due to regenerative braking.
                */
                if (motor_state.i_dq_a[1] * motor_state.v_dq_v[1] < 0.0f) {
                    msg.current = -msg.current;
                }

                msg.temperature = 273.15f + hal_get_temperature_degc();
                msg.rpm = int32_t(g_motor_state.angular_velocity_rad_per_s *
                                  to_rpm);
                msg.power_rating_pct = uint8_t(100.0f * power_w /
                                               (motor_params.max_current_a *
                                                motor_params.max_voltage_v));
                msg.esc_index = esc_index;

                broadcast_manager.encode_message(
                    esc_status_transfer_id++, msg);
                esc_status_time = current_time;
            } else if (current_time - node_status_time >=
                        node_status_interval) {
                uavcan::protocol::NodeStatus msg;

                msg.uptime_sec = current_time / 1000u;
                msg.health = g_controller_state.fault ?
                    msg.HEALTH_CRITICAL :
                    msg.HEALTH_OK;
                msg.mode = msg.MODE_OPERATIONAL;
                msg.sub_mode = 0u;
                msg.vendor_specific_status_code =
                    (uint16_t)g_controller_state.mode;
                broadcast_manager.encode_message(
                    node_status_transfer_id++, msg);
                node_status_time = current_time;
            } else if (g_enumeration_active &&
                    std::abs(g_enumeration_velocity_rad_per_s) >
                        ENUMERATION_VELOCITY_THRESHOLD_RAD_PER_S) {
                /*
                Change rotation direction if the enumerated direction was
                negative
                */
                if (g_enumeration_velocity_rad_per_s < 0.0f) {
                    value = configuration.get_param_value_by_index(
                        PARAM_CONTROL_DIRECTION);
                    configuration.set_param_value_by_index(
                        PARAM_CONTROL_DIRECTION, value == 0.0f ? 1.0f : 0.0f);
                }

                uavcan::protocol::enumeration::Indication msg;
                msg.parameter_name = "esc_index";
                broadcast_manager.encode_message(
                    enumeration_indication_transfer_id++, msg);

                /*
                Reset enumeration velocity to avoid sending another message
                straight away
                */
                g_enumeration_active = false;
                g_enumeration_velocity_rad_per_s = 0.0f;
            }
        }

        /* Transmit broadcast CAN frames if available */
        if (can_is_ready(0u) &&
                broadcast_manager.transmit_frame(message_id, length, message)) {
            can_tx(0u, message_id, length, message);
        }

        /*
        Update the controller mode and setpoint, unless the motor is currently
        stopping or enumeration is active.
        */
        if (!g_controller_state.fault && got_setpoint &&
                g_controller_state.mode != CONTROLLER_ABORTING &&
                !g_enumeration_active) {
            if (mode == CONTROLLER_VOLTAGE) {
                g_controller_state.voltage_setpoint = setpoint;
                g_controller_state.mode = CONTROLLER_VOLTAGE;
                last_setpoint_update = current_time;
            } else if (g_controller_state.mode == CONTROLLER_IDLING) {
                /*
                Setpoint was non-zero, but was not high enough to start
                spinning and we're not already running -- so start idling, or
                keep on idling.
                */
                g_controller_state.mode = CONTROLLER_IDLING;
                g_controller_state.voltage_setpoint = 0.0f;
                last_setpoint_update = current_time;
            }
        } else if (g_controller_state.mode != CONTROLLER_STOPPED &&
                   g_controller_state.mode != CONTROLLER_STOPPING &&
                   g_controller_state.mode != CONTROLLER_ABORTING &&
                   current_time - last_setpoint_update > THROTTLE_TIMEOUT_MS) {
            /*
            Stop gracefully if the present command mode doesn't provide an
            updated setpoint in the required period.
            */
            g_controller_state.mode = CONTROLLER_STOPPING;
            g_controller_state.voltage_setpoint = 0.0f;
        }

        /*
        Update the Kv parameter with the latest estimate of phi -- we don't
        really need to do this in the main loop, but it can't hurt.
        */
        if (g_phi_v_s_per_rad > 0.0f &&
                g_controller_state.mode == CONTROLLER_VOLTAGE) {
            configuration.set_param_value_by_index(
                PARAM_MOTOR_KV, to_rpm / g_phi_v_s_per_rad);
        }

        /* Stop enumeration if the deadline has expired */
        if (enumeration_deadline < current_time) {
            g_enumeration_active = false;
        }

        /*
        Only restart into the bootloader if the acknowledgement message has
        been sent, and we're otherwise unoccupied.
        */
        if (g_controller_state.mode == CONTROLLER_STOPPED &&
                broadcast_manager.is_tx_done() && can_is_ready(0u) &&
                service_manager.is_tx_done() && can_is_ready(1u) &&
                wants_bootloader_restart) {
            g_controller_state.fault = true;
            hal_restart();
        }
    }
}


int main(void) {
    struct control_params_t control_params;
    struct motor_params_t motor_params;
    float temp;
    uint8_t node_id;

    hal_reset();
    hal_set_low_frequency_callback(systick_cb);

    /* Read parameters from flash */
    Configuration configuration;
    configuration.read_motor_params(motor_params);
    configuration.read_control_params(control_params);

    /* Set phase reverse if required */
    if (configuration.get_param_value_by_index(PARAM_CONTROL_DIRECTION) == 1.0f) {
        hal_set_pwm_reverse(true);
    }

    /* Estimate motor parameters */
    hal_set_pwm_state(HAL_PWM_STATE_RUNNING);

    g_controller_state.mode = CONTROLLER_IDENTIFYING;
    g_parameter_estimator.start_estimation(hal_control_t_s);

    hal_set_high_frequency_callback(identification_cb);
    while (g_controller_state.mode == CONTROLLER_IDENTIFYING);
    hal_set_high_frequency_callback(NULL);

    /* Copy measured Rs and Ls */
    if (!g_controller_state.fault) {
        motor_params.rs_r = g_measured_rs_r;
        motor_params.ls_h = g_measured_ls_h;

        /*
        This is not strictly necessary as the values are never written to
        flash, but it does enable the measurements to be read via the
        parameter interfaces.
        */
        configuration.set_param_value_by_index(PARAM_MOTOR_RS,
                                               motor_params.rs_r);
        configuration.set_param_value_by_index(PARAM_MOTOR_LS,
                                               motor_params.ls_h);

        /* R/L * t must be < 5.0; R must be < 10.0 */
        if (motor_params.rs_r / motor_params.ls_h * hal_control_t_s >= 5.0f ||
                motor_params.rs_r > 20.0f) {
            g_controller_state.fault = true;
        }
    }

    /* Initialize the system with the motor parameters */
    ((volatile StateEstimator*)&g_estimator)->set_control_params(
        control_params.bandwidth_hz, hal_control_t_s);
    ((volatile StateEstimator*)&g_estimator)->set_motor_params(
        motor_params.rs_r, motor_params.ls_h, motor_params.phi_v_s_per_rad,
        hal_control_t_s);
    ((volatile StateEstimator*)&g_estimator)->reset_state();

    ((volatile DQCurrentController*)&g_current_controller)->set_params(
        motor_params, control_params, hal_control_t_s);

    /* Set up the control parameters */
    *((volatile float*)&g_controller_constants.accel_current_a) =
        motor_params.accel_voltage_v / motor_params.rs_r;
    *((volatile float*)&g_controller_constants.max_current_a) =
        motor_params.max_current_a;
    *((volatile float*)&g_controller_constants.max_voltage_v) =
        motor_params.max_voltage_v;
    *((volatile float*)&g_controller_constants.braking_current_a) =
        motor_params.max_current_a * control_params.braking_frac;
    *((volatile float*)&g_controller_constants.effective_inv_r) =
        control_params.gain / motor_params.rs_r;
    *((volatile float*)&g_controller_constants.max_vbus_v) =
        g_vbus_v;

    temp = 1.0f / (float(2.0 * M_PI) * control_params.bandwidth_hz);
    *((volatile float*)&g_controller_constants.control_lpf_coeff) =
        hal_control_t_s / (hal_control_t_s + temp);

    /* Clear audio state */
    g_audio_state.off_time = 0;
    g_audio_state.phase_rad = 0.0f;
    g_audio_state.angular_velocity_rad_per_u = 0.0f;
    g_audio_state.volume_v = 0.0f;

    /*
    After starting the ISR tasks we are no longer able to access g_estimator,
    g_current_controller and g_speed_controller, since they're updated from
    the ISRs and not declared volatile.
    */
    hal_set_high_frequency_callback(control_cb);

    node_id = hal_get_can_node_id();
    node_init(node_id, configuration);
    node_run(node_id, configuration, motor_params, control_params);
}
