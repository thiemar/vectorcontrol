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


enum controller_mode_t {
    CONTROLLER_STOPPED,
    CONTROLLER_IDENTIFYING,
    CONTROLLER_SPEED,
    CONTROLLER_POWER,
    CONTROLLER_STOPPING
};

struct controller_state_t {
    uint32_t time;
    uint32_t last_setpoint_update;
    float power_setpoint;
    float speed_setpoint;
    float current_setpoint;
    float max_speed_rad_per_s;
    float min_speed_rad_per_s;
    float max_current_a;
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

/* Motor parameter estimation state -- only used on startup */
volatile static float g_i_samples[4];
volatile static float g_v_samples[4];
volatile static bool g_parameter_estimator_done;


/* Written to the firmware image in post-processing */
extern volatile struct bootloader_app_descriptor flash_app_descriptor;


/*
Throttle timeout -- if we don't receive a setpoint update in this long,
we spin the motor down
*/
#define THROTTLE_TIMEOUT_MS 500u


/*
Hard current limit -- exceeding this shuts down the motor immediately
(possibly destroying the power stage depending on the reason for the
over-current)
*/
#define HARD_CURRENT_LIMIT_A 70.0f


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


static void node_init(uint8_t node_id);
static void node_run(
    uint8_t node_id,
    Configuration& configuration,
    const struct motor_params_t& motor_params
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
    float v_dq_v[2], power_setpoint, speed_setpoint,
          phase, audio_v, current_setpoint;
    struct motor_state_t motor_state;
    struct controller_state_t controller_state;

    controller_state =
        const_cast<struct controller_state_t&>(g_controller_state);

    /* Calculate audio component */
    if (g_audio_state.off_time) {
        phase = g_audio_state.phase_rad;
        phase += g_audio_state.angular_velocity_rad_per_u;
        if (phase > (float)M_PI) {
            phase -= (float)(2.0 * M_PI);
        }
        audio_v = g_audio_state.volume_v * (phase > 0.0f ? 1.0f : -1.0f); /*fast_sin(phase)*/
        g_audio_state.phase_rad = phase;
        g_audio_state.off_time--;
    } else {
        audio_v = 0.0f;
    }

    /* Update the state estimate with those values */
    g_estimator.update_state_estimate(
        i_ab_a, last_v_ab_v,
        controller_state.speed_setpoint > 0.0f ? 1.0 : -1.0f);
    g_estimator.get_state_estimate(motor_state);

    /*
    Handle stop condition -- set Vd, Vq to zero and hope the resulting hard
    stop doesn't break anything. Trigger at current limit.
    */
    if (i_ab_a[0] * i_ab_a[0] + i_ab_a[1] * i_ab_a[1] >
            HARD_CURRENT_LIMIT_A * HARD_CURRENT_LIMIT_A ||
            g_controller_state.fault) {
        g_controller_state.fault = true;
        g_controller_state.mode = controller_state.mode = CONTROLLER_STOPPED;
    }

    if (controller_state.mode == CONTROLLER_STOPPED) {
        g_v_dq_v[0] = v_dq_v[0] = 0.0f;
        g_estimator.reset_state();
        g_current_controller.reset_state();
        g_speed_controller.reset_state();

        g_v_dq_v[1] = v_dq_v[1] = audio_v;
        g_estimator.get_est_v_alpha_beta_from_v_dq(out_v_ab_v, v_dq_v);
    } else {
        /* Update the speed controller setpoint */
        if (controller_state.mode == CONTROLLER_POWER) {
            /*
            In torque control mode, the torque input is used to limit the
            speed controller's output, and the speed controller setpoint
            always requests acceleration or deceleration in the direction of
            the requested torque.
            */
            power_setpoint = g_controller_state.power_setpoint;

            g_controller_state.speed_setpoint = speed_setpoint =
                motor_state.angular_velocity_rad_per_s;
            g_speed_controller.set_power_setpoint(power_setpoint);
        } else if (controller_state.mode == CONTROLLER_STOPPING) {
            /*
            When stopping, the speed controller setpoint tracks the current
            angular velocity
            */
            g_controller_state.speed_setpoint = speed_setpoint =
                motor_state.angular_velocity_rad_per_s;
            g_speed_controller.set_speed_setpoint(speed_setpoint);
        } else /* controller_state.mode == CONTROLLER_SPEED */ {
            speed_setpoint = g_controller_state.speed_setpoint;
            g_speed_controller.set_speed_setpoint(speed_setpoint);
        }

        /*
        Update the current controller with the output of the speed controller
        when in speed control mode, or a constant opposing torque when
        stopping.
        */
        current_setpoint = g_speed_controller.update(motor_state);

        if (controller_state.mode == CONTROLLER_STOPPING) {
            /*
            Add a small amount of negative torque to ensure the motor actually
            shuts down.
            */
            current_setpoint = speed_setpoint > 0.0f ? -0.25f : 0.25f;
        } else {
            g_controller_state.current_setpoint = current_setpoint;
        }

        /* Update the stator current controller setpoint. */
        g_current_controller.set_setpoint(current_setpoint);

        /* Run the current controller and obtain new Vd and Vq outputs. */
        g_current_controller.update(v_dq_v,
                                    motor_state.i_dq_a,
                                    motor_state.angular_velocity_rad_per_s,
                                    vbus_v,
                                    audio_v);

        /* Save global voltages */
        g_v_dq_v[0] = v_dq_v[0];
        g_v_dq_v[1] = v_dq_v[1];

        /*
        Transform Vd and Vq into the alpha-beta frame, and set the PWM output
        accordingly.
        */
        g_estimator.get_est_v_alpha_beta_from_v_dq(out_v_ab_v, v_dq_v);
    }

    /*
    Could look at low-passing these too, based on the FOC status output rate
    */
    g_motor_state.angular_acceleration_rad_per_s2 =
        motor_state.angular_acceleration_rad_per_s2;
    g_motor_state.angular_velocity_rad_per_s =
        motor_state.angular_velocity_rad_per_s;
    g_motor_state.angle_rad = motor_state.angle_rad;
    g_motor_state.i_dq_a[0] = motor_state.i_dq_a[0];
    g_motor_state.i_dq_a[1] = motor_state.i_dq_a[1];
    g_vbus_v = vbus_v;
}


void systick_cb(void) {
    uint32_t state_time, state_last_setpoint_update;
    enum controller_mode_t state_mode;

    state_mode = g_controller_state.mode;
    state_time = g_controller_state.time + 1u;
    state_last_setpoint_update = g_controller_state.last_setpoint_update;

    /* Motor shutdown logic -- stop when back EMF drops below 0.1 V */
    if (std::abs(g_v_dq_v[0]) < 0.1f && std::abs(g_v_dq_v[1]) < 0.1f &&
            state_mode == CONTROLLER_STOPPING) {
        g_controller_state.mode = CONTROLLER_STOPPED;
        g_controller_state.speed_setpoint = 0.0f;
        g_controller_state.current_setpoint = 0.0f;
    } else if ((state_mode == CONTROLLER_SPEED ||
                state_mode == CONTROLLER_POWER) &&
                state_time - state_last_setpoint_update > THROTTLE_TIMEOUT_MS) {
        /*
        Stop gracefully if the present command mode doesn't provide an
        updated setpoint in the required period.
        */
        g_controller_state.mode = CONTROLLER_STOPPING;
    }

    g_controller_state.time = state_time;
}


static void node_init(uint8_t node_id) {
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE, true,
        uavcan::protocol::param::ExecuteOpcode::DefaultDataTypeID,
        node_id);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_GETSET, true,
        uavcan::protocol::param::GetSet::DefaultDataTypeID,
        node_id);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE, true,
        uavcan::protocol::file::BeginFirmwareUpdate::DefaultDataTypeID,
        node_id);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_GETNODEINFO, true,
        uavcan::protocol::GetNodeInfo::DefaultDataTypeID,
        node_id);
    hal_set_can_dtid_filter(
        1u, UAVCAN_PROTOCOL_RESTARTNODE, true,
        uavcan::protocol::RestartNode::DefaultDataTypeID,
        node_id);

    hal_set_can_dtid_filter(
        0u, UAVCAN_EQUIPMENT_ESC_RAWCOMMAND, false,
        uavcan::equipment::esc::RawCommand::DefaultDataTypeID,
        node_id);
    hal_set_can_dtid_filter(
        0u, UAVCAN_EQUIPMENT_ESC_RPMCOMMAND, false,
        uavcan::equipment::esc::RPMCommand::DefaultDataTypeID,
        node_id);
    hal_set_can_dtid_filter(
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
             foc_status_interval, last_tx_time;
    uint8_t filter_id, foc_status_transfer_id, node_status_transfer_id,
            esc_status_transfer_id, esc_index, message[8];
    enum hal_status_t status;
    enum controller_mode_t mode;
    float setpoint, value, is_a, vs_v, v_dq_v[2];
    bool got_setpoint, param_valid, wants_bootloader_restart;
    struct param_t param;
    struct motor_state_t motor_state;
    uint16_t foc_status_dtid;

    foc_status_transfer_id = node_status_transfer_id =
        esc_status_transfer_id = 0u;

    foc_status_time = node_status_time = esc_status_time = last_tx_time = 0u;

    wants_bootloader_restart = false;

    esc_index = (uint8_t)
        configuration.get_param_value_by_index(PARAM_UAVCAN_ESC_INDEX);
    foc_status_dtid = (uint16_t)
        configuration.get_param_value_by_index(PARAM_FOC_ESCSTATUS_ID);

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
        PARAM_FOC_ESCSTATUS_INTERVAL) * 0.001f);

    g_controller_state.min_speed_rad_per_s = motor_params.min_speed_rad_per_s;
    g_controller_state.max_speed_rad_per_s = motor_params.max_speed_rad_per_s;
    g_controller_state.max_current_a = motor_params.max_current_a;

    while (true) {
        current_time = g_controller_state.time;
        got_setpoint = false;
        setpoint = 0.0f;

        /*
        Check for UAVCAN commands (FIFO 0) -- these are all broadcasts
        */
        do {
            status = hal_receive_can_message(0u, &filter_id, &message_id,
                                         &length, message);
            /* Filter IDs are per-FIFO, so convert this back to the bank index */
            filter_id = (uint8_t)(filter_id + UAVCAN_EQUIPMENT_ESC_RAWCOMMAND);
            if (status == HAL_STATUS_OK) {
                uavcan::TransferCRC crc;
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
            }
        } while (status == HAL_STATUS_OK && !broadcast_manager.is_rx_done());

        if (broadcast_manager.is_rx_done()) {
            if (filter_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND &&
                    broadcast_manager.decode_esc_rawcommand(raw_cmd) &&
                    esc_index < raw_cmd.cmd.size()) {
                got_setpoint = true;
                mode = CONTROLLER_POWER;
                /* Scale 0-8191 to represent 0 to maximum power. */
                setpoint = (float)raw_cmd.cmd[esc_index] * (1.0f / 8191.0f) *
                           (motor_params.max_voltage_v *
                            motor_params.max_current_a);
            } else if (filter_id == UAVCAN_EQUIPMENT_ESC_RPMCOMMAND &&
                        broadcast_manager.decode_esc_rpmcommand(rpm_cmd) &&
                        esc_index < rpm_cmd.rpm.size()) {
                got_setpoint = true;
                mode = CONTROLLER_SPEED;
                setpoint = (float)rpm_cmd.rpm[esc_index] * (1.0f / 60.0f) *
                          (float)motor_params.num_poles * (float)M_PI;
            } else if (filter_id == UAVCAN_EQUIPMENT_INDICATION_BEEPCOMMAND &&
                       broadcast_manager.decode_indication_beepcommand(beep_cmd)) {
                /* Set up audio generation -- aim for 0.5 A */
                g_audio_state.off_time =
                    (uint32_t)(beep_cmd.duration / hal_control_t_s);
                g_audio_state.angular_velocity_rad_per_u =
                    2.0f * (float)M_PI * hal_control_t_s *
                    std::max(100.0f, beep_cmd.frequency);
                g_audio_state.volume_v = std::min(
                    2.0f * motor_params.rs_r,
                    0.5f * std::min((float)g_vbus_v, motor_params.max_voltage_v));
            }
        }

        /*
        Check for UAVCAN service requests (FIFO 1) -- only process if the
        first byte of the data is the local node ID
        */
        do {
            status = hal_receive_can_message(1u, &filter_id, &message_id,
                                             &length, message);
            if (status == HAL_STATUS_OK) {
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
            }
        } while (status == HAL_STATUS_OK && !service_manager.is_rx_done());

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
                        value = (float)gs_req.value.to<uavcan::protocol::param::Value::Tag::integer_value>();
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
                        resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int64_t)value;
                        resp.default_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int64_t)param.default_value;
                        resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int64_t)param.min_value;
                        resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int64_t)param.max_value;
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
                resp.hardware_version.major = 1u;
                resp.hardware_version.minor = 0u;
                /* Set the unique ID */
                memset(resp.hardware_version.unique_id.begin(), 0u,
                       resp.hardware_version.unique_id.size());
                memcpy(resp.hardware_version.unique_id.begin(),
                       (uint8_t*)0x1ffff7ac, 12u);
                /* Set the hardware name */
                resp.name = "com.thiemar.s2740vc-v1";

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

        /* Transmit service responses if available */
        if (current_time > last_tx_time + 1u && hal_is_can_ready(1u) &&
                service_manager.transmit_frame(message_id, length, message)) {
            (void)hal_transmit_can_message(1u, message_id, length, message);
            last_tx_time = current_time;
        }

        if (broadcast_manager.is_tx_done() && service_manager.is_tx_done() &&
                !service_manager.is_rx_in_progress(current_time)) {
            motor_state = const_cast<struct motor_state_t&>(g_motor_state);
            v_dq_v[0] = g_v_dq_v[0];
            v_dq_v[1] = g_v_dq_v[1];

            if (foc_status_interval && current_time - foc_status_time >=
                    foc_status_interval) {
                uavcan::equipment::esc::FOCStatus msg;

                msg.i_dq[0] = motor_state.i_dq_a[0];
                msg.i_dq[1] = motor_state.i_dq_a[1];
                msg.i_setpoint = g_controller_state.current_setpoint;
                msg.v_dq[0] = v_dq_v[0];
                msg.v_dq[1] = v_dq_v[1];
                /* Acceleration */
                msg.hfi_dq[0] = motor_state.angular_acceleration_rad_per_s2 *
                    60.0f / ((float)M_PI * (float)motor_params.num_poles);
                /* Power */
                msg.hfi_dq[1] = 3.0f * motor_params.phi_v_s_per_rad *
                                motor_state.i_dq_a[1] *
                                motor_state.angular_velocity_rad_per_s -
                                motor_params.rotor_inertia_kg_m2 *
                                motor_state.angular_velocity_rad_per_s *
                                motor_state.angular_acceleration_rad_per_s2 *
                                (2.0f / (float)motor_params.num_poles) *
                                (2.0f / (float)motor_params.num_poles);
                msg.consistency = 0;
                msg.vbus = g_vbus_v;
                msg.temperature = 273.15f + hal_get_temperature_degc();
                msg.angle = (uint8_t)
                    (motor_state.angle_rad * (0.5f / M_PI) * 255.0f);
                msg.rpm = motor_state.angular_velocity_rad_per_s *
                    60.0f / ((float)M_PI * (float)motor_params.num_poles);
                msg.rpm_setpoint = g_controller_state.speed_setpoint *
                    60.0f / ((float)M_PI * (float)motor_params.num_poles);
                msg.esc_index = esc_index;
                broadcast_manager.encode_foc_status(
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
                msg.health = g_controller_state.fault ?
                    msg.HEALTH_CRITICAL :
                    msg.HEALTH_OK;
                msg.mode = msg.MODE_OPERATIONAL;
                msg.sub_mode = 0u;
                msg.vendor_specific_status_code =
                    (uint16_t)g_controller_state.mode;
                broadcast_manager.encode_nodestatus(
                    node_status_transfer_id++, msg);
                node_status_time = current_time;
            }
        }

        /* Transmit broadcast CAN frames if available */
        if (current_time > last_tx_time + 1u && hal_is_can_ready(0u) &&
                broadcast_manager.transmit_frame(message_id, length, message)) {
            (void)hal_transmit_can_message(0u, message_id, length, message);
            last_tx_time = current_time;
        }

        /* Update the controller mode and setpoint */
        if (!g_controller_state.fault && got_setpoint) {
            if (mode == CONTROLLER_SPEED &&
                    std::abs(setpoint) > 2.0f * (float)M_PI) {
                g_controller_state.speed_setpoint = setpoint;
            } else if (mode == CONTROLLER_POWER &&
                    std::abs(setpoint) >= 0.1f) {
                g_controller_state.power_setpoint = setpoint;
            } else {
                mode = CONTROLLER_STOPPING;
            }
            g_controller_state.mode = mode;
            g_controller_state.last_setpoint_update = current_time;
        }

        /*
        Only restart into the bootloader if the acknowledgement message has
        been sent, and we're otherwise unoccupied.
        */
        if (g_controller_state.mode == CONTROLLER_STOPPED &&
                broadcast_manager.is_tx_done() &&
                service_manager.is_tx_done() && wants_bootloader_restart) {
            hal_restart();
        }
    }
}


int __attribute__((externally_visible,noreturn)) main(void) {
    Configuration configuration;
    struct control_params_t control_params;
    struct motor_params_t motor_params;
    uint32_t i;
    float i_samples[4], v_samples[4], rs_r, ls_h, limit_speed;
    uint8_t node_id;

    hal_reset();
    hal_set_pwm_state(HAL_PWM_STATE_LOW);

    /* Read parameters from flash */
    configuration.read_motor_params(motor_params);
    configuration.read_control_params(control_params);

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
    g_speed_controller.set_current_limit_a(motor_params.max_current_a);

    /*
    Calculate the motor's maximum (electrical) speed based on Vbus at startup
    time, the maximum modulation limit, and the motor's flux linkage.
    */
    limit_speed = std::min(0.95f * g_vbus_v, motor_params.max_voltage_v) /
                  motor_params.phi_v_s_per_rad;
    if (motor_params.max_speed_rad_per_s > limit_speed) {
        motor_params.max_speed_rad_per_s = limit_speed;
    }

    /* Clear audio state */
    g_audio_state.off_time = 0;
    g_audio_state.phase_rad = 0.0f;
    g_audio_state.angular_velocity_rad_per_u = 0.0f;
    g_audio_state.volume_v = 0.0f;

    /* Start PWM */
    hal_set_pwm_state(HAL_PWM_STATE_RUNNING);

    /*
    After starting the ISR tasks we are no longer able to access g_estimator,
    g_current_controller and g_speed_controller, since they're updated from
    the ISRs and not declared volatile.
    */
    hal_set_high_frequency_callback(control_cb);
    hal_set_low_frequency_callback(systick_cb);

    hal_enable_can_transmit();

    node_id = hal_get_can_node_id();
    node_init(node_id);
    node_run(node_id, configuration, motor_params);
}
