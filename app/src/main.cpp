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
    UAVCAN_EQUIPMENT_ESC_RAWCOMMAND,
    UAVCAN_EQUIPMENT_ESC_RPMCOMMAND,
    UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND,
    THIEMAR_EQUIPMENT_ESC_THRUSTPOWERCOMMAND
};


/*
"Owned" by identification_cb / control_cb (only one will be running at a given
stage in execution).
*/
static StateEstimator g_estimator;
static ParameterEstimator g_parameter_estimator;
static DQCurrentController g_current_controller;
static SpeedController g_speed_controller;


enum controller_mode_t {
    CONTROLLER_STOPPED,
    CONTROLLER_IDENTIFYING,
    CONTROLLER_SPEED,
    CONTROLLER_POWER,
    CONTROLLER_STOPPING,
    CONTROLLER_IDLING
};

struct controller_state_t {
    uint32_t time;
    float power_setpoint;
    float speed_setpoint;
    float internal_speed_setpoint;
    float current_setpoint;
    float closed_loop_frac;
    float idle_speed_rad_per_s;
    float spinup_rate_rad_per_s2;
    float accel_current_a;
    enum controller_mode_t mode;
    bool fault;
};

struct audio_state_t {
    uint32_t off_time;
    float phase_rad;
    float angular_velocity_rad_per_u;
    float volume_v;
};


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
volatile static float g_v_dq_v[2];
volatile static float g_phi_v_s_per_rad;


/* Motor parameter estimation state -- only used on startup */
volatile static float g_measured_rs_r;
volatile static float g_measured_ls_h;


/* Written to the firmware image in post-processing */
extern volatile struct bootloader_app_descriptor flash_app_descriptor;


/*
Throttle timeout -- if we don't receive a setpoint update in this long,
we spin the motor down
*/
#define THROTTLE_TIMEOUT_MS 200u


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
    const struct motor_params_t& motor_params
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
    float v_dq_v[2], phase, audio_v, current_setpoint, power_setpoint,
          speed_setpoint, internal_speed_setpoint, closed_loop_frac,
          idle_speed_rad_per_s, accel_current_a, spinup_rate_rad_per_s2,
          new_closed_loop_frac, phi;
    struct motor_state_t motor_state;
    enum controller_mode_t mode;

    mode = g_controller_state.mode;
    power_setpoint = g_controller_state.power_setpoint;
    speed_setpoint = g_controller_state.speed_setpoint;
    internal_speed_setpoint = g_controller_state.internal_speed_setpoint;
    closed_loop_frac = g_controller_state.closed_loop_frac;

    idle_speed_rad_per_s = g_controller_state.idle_speed_rad_per_s;
    accel_current_a = g_controller_state.accel_current_a;
    spinup_rate_rad_per_s2 = g_controller_state.spinup_rate_rad_per_s2;

    v_dq_v[0] = g_v_dq_v[0];
    v_dq_v[1] = g_v_dq_v[1];
    audio_v = 0.0f;

    /* Update the state estimate with the latest measurements */
    g_estimator.update_state_estimate(i_ab_a, last_v_ab_v,
                                      internal_speed_setpoint,
                                      closed_loop_frac);
    g_estimator.get_state_estimate(motor_state);

    /* Get the latest phi estimate */
    phi = g_estimator.get_phi_estimate();
    g_speed_controller.set_phi_v_s_per_rad(phi);

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
    if (mode == CONTROLLER_STOPPING) {
        /*
        When stopping, the speed controller setpoint tracks the current
        angular velocity.
        */
        internal_speed_setpoint = motor_state.angular_velocity_rad_per_s;
        g_speed_controller.set_speed_setpoint(internal_speed_setpoint);

        /* Stop when back EMF drops below 0.1 V */
        if (std::abs(v_dq_v[0]) < 0.1f && std::abs(v_dq_v[1]) < 0.1f) {
            g_controller_state.mode = CONTROLLER_STOPPED;
        }
    } else if (mode == CONTROLLER_IDLING) {
        /* Rotate at the idle speed */
        internal_speed_setpoint = idle_speed_rad_per_s;
        g_speed_controller.set_speed_setpoint(internal_speed_setpoint);
    } else if ((mode == CONTROLLER_POWER || mode == CONTROLLER_SPEED) &&
                closed_loop_frac < 1.0f) {
        /*
        In power or speed control mode, but not yet spinning fast enough
        for closed-loop control -- spin up gradually until we reach the
        minimum closed-loop speed.
        */
        internal_speed_setpoint +=
            spinup_rate_rad_per_s2 * hal_control_t_s *
            (speed_setpoint > 0.0f || power_setpoint > 0.0f ? 1.0f : -1.0f);
        g_speed_controller.set_speed_setpoint(internal_speed_setpoint);
    } else if (mode == CONTROLLER_POWER) {
        /*
        In torque control mode, the torque input is used to limit the
        speed controller's output, and the speed controller setpoint
        always requests acceleration or deceleration in the direction of
        the requested torque.
        */
        internal_speed_setpoint = motor_state.angular_velocity_rad_per_s;
        g_speed_controller.set_power_setpoint(power_setpoint);
    } else if (mode == CONTROLLER_SPEED) {
        internal_speed_setpoint = speed_setpoint;
        g_speed_controller.set_speed_setpoint(internal_speed_setpoint);
    } else /* Normally: if (mode == CONTROLLER_STOPPED), but catch-all */ {
        internal_speed_setpoint = 0.0f;
        g_speed_controller.reset_state();
    }

    /*
    Update the current controller with the output of the speed controller
    when in speed control mode, or a constant opposing torque when
    stopping.
    */
    current_setpoint = g_speed_controller.update(motor_state);

    /*
    Inner control loop -- adjust motor voltage to maintain the current
    setpoint
    */
    if (mode == CONTROLLER_STOPPING) {
        /*
        Add a small amount of negative torque to ensure the motor actually
        shuts down.
        */
        current_setpoint = internal_speed_setpoint > 0.0f ? -0.25f : 0.25f;
    } else if (mode == CONTROLLER_IDLING) {
        closed_loop_frac = 0.0f;
        current_setpoint = internal_speed_setpoint > 0.0f ?
                           accel_current_a : -accel_current_a;
    } else if (mode == CONTROLLER_POWER || mode == CONTROLLER_SPEED) {
        /*
        Interpolate between the initial acceleration current and the speed
        controller's current output as we transition out of open-loop mode.
        */
        current_setpoint += (1.0f - closed_loop_frac) *
                            ((internal_speed_setpoint > 0.0f ?
                                accel_current_a : -accel_current_a) -
                             current_setpoint);
    } else {
        g_current_controller.reset_state();
        closed_loop_frac = 0.0f;
        current_setpoint = 0.0f;
    }

    /* Update the stator current controller setpoint. */
    g_current_controller.set_setpoint(current_setpoint);

    /* Run the current controller and obtain new Vd and Vq outputs. */
    g_current_controller.update(v_dq_v,
                                motor_state.i_dq_a,
                                motor_state.angular_velocity_rad_per_s,
                                vbus_v,
                                audio_v);

    /*
    Work out the transition value between open-loop and closed-loop operation
    based on the square of the output voltage. Smooth the transition out quite
    a bit to avoid false triggering.
    */
    new_closed_loop_frac = 8.0f * (v_dq_v[0] * v_dq_v[0] +
                            (v_dq_v[1] - audio_v) * (v_dq_v[1] - audio_v)) -
                           7.0f;
    if (new_closed_loop_frac < 0.0f) {
        new_closed_loop_frac = 0.0f;
    }

    if (closed_loop_frac < 1.0f) {
        closed_loop_frac +=
            (new_closed_loop_frac - closed_loop_frac) * 0.0001f;
    }
    if (closed_loop_frac > 1.0f) {
        closed_loop_frac = 1.0f;
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
    g_controller_state.closed_loop_frac = closed_loop_frac;
    g_controller_state.current_setpoint = current_setpoint;

    g_motor_state.angular_acceleration_rad_per_s2 =
        motor_state.angular_acceleration_rad_per_s2;
    g_motor_state.angular_velocity_rad_per_s =
        motor_state.angular_velocity_rad_per_s;
    g_motor_state.angle_rad = motor_state.angle_rad;
    g_motor_state.i_dq_a[0] = motor_state.i_dq_a[0];
    g_motor_state.i_dq_a[1] = motor_state.i_dq_a[1];
    g_v_dq_v[0] = v_dq_v[0];
    g_v_dq_v[1] = v_dq_v[1];
    g_vbus_v = vbus_v;
    g_phi_v_s_per_rad = phi;
}


void systick_cb(void) {
    g_controller_state.time++;
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
        0u, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND, false,
        uavcan::equipment::esc::RawCommand::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        0u, UAVCAN_EQUIPMENT_ESC_RPMCOMMAND, false,
        uavcan::equipment::esc::RPMCommand::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        0u, UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND, false,
        uavcan::equipment::indication::BeepCommand::DefaultDataTypeID,
        node_id);
}


static void __attribute__((noreturn)) node_run(
    uint8_t node_id,
    Configuration& configuration,
    const struct motor_params_t& motor_params
) {
    size_t length, i;
    uint32_t message_id, current_time, node_status_time, esc_status_time,
             foc_status_time, node_status_interval, esc_status_interval,
             foc_status_interval, last_setpoint_update;
    uint8_t filter_id, foc_status_transfer_id, node_status_transfer_id,
            esc_status_transfer_id, esc_index, message[8],
            broadcast_filter_id, service_filter_id;
    enum controller_mode_t mode;
    float setpoint, value, is_a, vs_v, v_dq_v[2], inv_num_poles;
    bool got_setpoint, param_valid, wants_bootloader_restart;
    struct param_t param;
    struct motor_state_t motor_state;
    uint16_t foc_status_dtid;

    foc_status_transfer_id = node_status_transfer_id =
        esc_status_transfer_id = 0u;

    broadcast_filter_id = service_filter_id = 0xFFu;

    foc_status_time = node_status_time = esc_status_time =
        last_setpoint_update = 0u;

    wants_bootloader_restart = false;

    esc_index = (uint8_t)
        configuration.get_param_value_by_index(PARAM_UAVCAN_ESC_INDEX);
    foc_status_dtid = (uint16_t)
        configuration.get_param_value_by_index(PARAM_THIEMAR_STATUS_ID);

    UAVCANTransferManager broadcast_manager(node_id);
    UAVCANTransferManager service_manager(node_id);

    uavcan::equipment::esc::RawCommand raw_cmd;
    uavcan::equipment::esc::RPMCommand rpm_cmd;
    uavcan::equipment::indication::BeepCommand beep_cmd;
    uavcan::protocol::param::ExecuteOpcode::Request xo_req;
    uavcan::protocol::param::GetSet::Request gs_req;
    uavcan::protocol::RestartNode::Request rn_req;

    node_status_interval = 900u;
    esc_status_interval = (uint32_t)(configuration.get_param_value_by_index(
        PARAM_UAVCAN_ESCSTATUS_INTERVAL) * 0.001f);
    foc_status_interval = (uint32_t)(configuration.get_param_value_by_index(
        PARAM_THIEMAR_STATUS_INTERVAL) * 0.001f);

    g_controller_state.idle_speed_rad_per_s =
        motor_params.idle_speed_rad_per_s;
    g_controller_state.spinup_rate_rad_per_s2 =
        motor_params.spinup_rate_rad_per_s2;
    g_controller_state.accel_current_a =
        motor_params.accel_voltage_v / motor_params.rs_r;

    inv_num_poles = 1.0f / float(motor_params.num_poles);

    while (true) {
        current_time = g_controller_state.time;
        got_setpoint = false;
        setpoint = 0.0f;

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
                case UAVCAN_EQUIPMENT_ESC_RPMCOMMAND:
                    crc = uavcan::equipment::esc::RPMCommand::getDataTypeSignature().toTransferCRC();
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
                    esc_index < raw_cmd.cmd.size()) {
                got_setpoint = true;
                mode = CONTROLLER_POWER;
                /* Scale 0-8191 to represent 0 to maximum power. */
                setpoint = float(raw_cmd.cmd[esc_index]) * float(1.0 / 8192.0) *
                           (/*motor_params.max_voltage_v * */
                            motor_params.max_current_a);
            } else if (broadcast_filter_id == UAVCAN_EQUIPMENT_ESC_RPMCOMMAND &&
                        broadcast_manager.decode(rpm_cmd) &&
                        esc_index < rpm_cmd.rpm.size()) {
                got_setpoint = true;
                mode = CONTROLLER_SPEED;
                setpoint = float(M_PI / 60.0) * float(rpm_cmd.rpm[esc_index] *
                                                      motor_params.num_poles);
            } else if (broadcast_filter_id == UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND &&
                       broadcast_manager.decode(beep_cmd)) {
                /* Set up audio generation -- aim for 0.5 A */
                g_audio_state.off_time =
                    (uint32_t)(beep_cmd.duration / hal_control_t_s);
                g_audio_state.angular_velocity_rad_per_u =
                    float(2.0 * M_PI) * hal_control_t_s *
                    std::max(100.0f, beep_cmd.frequency);
                g_audio_state.volume_v = 0.5f;
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
                    for (i = 0u; i < NUM_PARAMS; i++) {
                        configuration.get_param_by_index(param, (uint8_t)i);
                        configuration.set_param_value_by_index(
                            (uint8_t)i, param.default_value);
                    }
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
                    g_controller_state.fault = true;
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
            v_dq_v[0] = g_v_dq_v[0];
            v_dq_v[1] = g_v_dq_v[1];

            if (foc_status_interval && current_time - foc_status_time >=
                    foc_status_interval) {
                thiemar::equipment::esc::Status msg;

                msg.i_dq[0] = motor_state.i_dq_a[0];
                msg.i_dq[1] = motor_state.i_dq_a[1];
                msg.i_setpoint = g_controller_state.current_setpoint;

                msg.v_dq[0] = v_dq_v[0];
                msg.v_dq[1] = v_dq_v[1];

                msg.power = g_phi_v_s_per_rad *
                            motor_state.i_dq_a[1] *
                            motor_state.angular_velocity_rad_per_s -
                            motor_params.rotor_inertia_kg_m2 *
                            motor_state.angular_velocity_rad_per_s *
                            motor_state.angular_acceleration_rad_per_s2 *
                            (2.0f * inv_num_poles) * (2.0f * inv_num_poles);

                msg.acceleration =
                    motor_state.angular_acceleration_rad_per_s2 *
                    float(60.0 / M_PI) * inv_num_poles;
                msg.rpm = motor_state.angular_velocity_rad_per_s *
                    float(60.0 / M_PI) * inv_num_poles;
                msg.rpm_setpoint = g_controller_state.speed_setpoint *
                    float(60.0 / M_PI) * inv_num_poles;

                msg.esc_index = esc_index;

                broadcast_manager.encode_message(
                    foc_status_transfer_id++, foc_status_dtid, msg);
                foc_status_time = current_time;
            } else if (esc_status_interval &&
                    current_time - esc_status_time >= esc_status_interval) {
                uavcan::equipment::esc::Status msg;

                is_a = __VSQRTF(
                    motor_state.i_dq_a[0] * motor_state.i_dq_a[0] +
                    motor_state.i_dq_a[1] * motor_state.i_dq_a[1]);
                vs_v = __VSQRTF(v_dq_v[0] * v_dq_v[0] +
                                v_dq_v[1] * v_dq_v[1]);

                msg.voltage = g_vbus_v;
                msg.current = is_a * vs_v / g_vbus_v;
                msg.temperature = 273.15f + hal_get_temperature_degc();
                msg.rpm = int32_t(g_motor_state.angular_velocity_rad_per_s *
                                  float(60.0 / M_PI) * inv_num_poles);
                msg.power_rating_pct = uint8_t(100.0f * (is_a * vs_v) /
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
            }
        }

        /* Transmit broadcast CAN frames if available */
        if (can_is_ready(0u) &&
                broadcast_manager.transmit_frame(message_id, length, message)) {
            can_tx(0u, message_id, length, message);
        }

        /* Update the controller mode and setpoint */
        if (!g_controller_state.fault && got_setpoint) {
            if (mode == CONTROLLER_SPEED &&
                    std::abs(setpoint) >= 60.0f) {
                g_controller_state.speed_setpoint = setpoint;
                g_controller_state.mode = CONTROLLER_SPEED;
                last_setpoint_update = current_time;
            } else if (mode == CONTROLLER_POWER &&
                    std::abs(setpoint) >= 1.0f) {
                g_controller_state.power_setpoint = setpoint;
                g_controller_state.mode = CONTROLLER_POWER;
                last_setpoint_update = current_time;
            } else if ((g_controller_state.mode == CONTROLLER_STOPPED &&
                        motor_params.idle_speed_rad_per_s > 0.0f) ||
                        g_controller_state.mode == CONTROLLER_IDLING) {
                g_controller_state.mode = CONTROLLER_IDLING;
                g_controller_state.speed_setpoint =
                    g_controller_state.power_setpoint = 0.0f;
                last_setpoint_update = current_time;
            }
        } else if (g_controller_state.mode != CONTROLLER_STOPPED &&
                   g_controller_state.mode != CONTROLLER_STOPPING &&
                   current_time - last_setpoint_update > THROTTLE_TIMEOUT_MS) {
            /*
            Stop gracefully if the present command mode doesn't provide an
            updated setpoint in the required period.
            */
            g_controller_state.mode = CONTROLLER_STOPPING;
            g_controller_state.speed_setpoint =
                g_controller_state.power_setpoint = 0.0f;
        }

        /*
        Update the Kv parameter with the latest estimate of phi -- we don't
        really need to do this in the main loop, but it can't hurt.
        */
        if (g_phi_v_s_per_rad > 0.0f &&
                (g_controller_state.mode == CONTROLLER_SPEED ||
                 g_controller_state.mode == CONTROLLER_POWER)) {
            configuration.set_param_value_by_index(
                PARAM_MOTOR_KV,
                float(60.0 / M_PI) * inv_num_poles / g_phi_v_s_per_rad);
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

        /* R/L * t must be < 1.0 */
        if (motor_params.rs_r / motor_params.ls_h * hal_control_t_s >= 1.0f) {
            //g_controller_state.fault = true;
            motor_params.rs_r = 0.1f;
            motor_params.ls_h = 1e-5f;
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
    ((volatile SpeedController*)&g_speed_controller)->set_params(
        motor_params, control_params, hal_control_t_s);

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
    node_run(node_id, configuration, motor_params);
}
