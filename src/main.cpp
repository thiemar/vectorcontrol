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
#include "bootloader.h"
#include "can.h"
#include "park.h"
#include "perf.h"
#include "estimator.h"
#include "controller.h"
#include "configuration.h"

/*
"Owned" by identification_cb / control_cb (only one will be running at a given
stage in execution).
*/
static StateEstimator g_estimator;
static ParameterEstimator g_parameter_estimator;
static DQCurrentController g_current_controller;
static SpeedController g_speed_controller;
static AlignmentController g_alignment_controller;


/* "Owned" by systick_cb */
enum controller_mode_t {
    CONTROLLER_STOPPED,
    CONTROLLER_IDENTIFYING,
    CONTROLLER_SPEED,
    CONTROLLER_TORQUE,
    CONTROLLER_STOPPING
};
volatile static enum controller_mode_t g_controller_mode;
volatile static enum controller_mode_t g_pwm_controller_mode;
volatile static enum {
    INPUT_PWM,
    INPUT_CAN,
    INPUT_UAVCAN
} g_input_source;
volatile static float g_current_controller_setpoint;
volatile static float g_speed_controller_setpoint;
volatile static bool g_estimator_converged;
volatile static float g_estimator_hfi[3];


volatile static uint32_t g_time;
volatile static uint32_t g_last_pwm;
volatile static uint32_t g_valid_pwm;
volatile static bool g_valid_can;
volatile static uint32_t g_last_can;
volatile static uint32_t g_last_uavcan;
volatile static bool g_fault;
volatile static uint8_t g_can_node_id;


/*
Globally-visible board state updated by high-frequency callbacks and read by
the low-frequency callback (doesn't matter if it's slightly out of sync as
it's only used for reporting).
*/
volatile static struct motor_state_t g_motor_state;
volatile static float g_vbus_v;
volatile static float g_v_dq_v[2];
volatile static struct motor_params_t g_motor_params;
volatile static float g_i_samples[4];
volatile static float g_v_samples[4];
volatile static bool g_parameter_estimator_done;


/* "Owned" by can_rx_cb */
static Configuration g_configuration;


/* Mixed ownership */
static UAVCANServer g_uavcan_server(g_configuration);


/* Timeouts */
#define INTERFACE_TIMEOUT_MS 1500u
#define THROTTLE_TIMEOUT_MS 500u


/* Hard current limit -- exceeding this shuts down the motor */
#define HARD_CURRENT_LIMIT_A 60.0f


/* RC PWM minimum and maximum limits */
#define RC_PWM_ARM_PULSES 10u
#define RC_PWM_MIN_WIDTH_US 950u
#define RC_PWM_ZERO_WIDTH_US 1100u
#define RC_PWM_FS_WIDTH_US 1850u
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
extern "C" bool can_esc_command_cb(
    enum uavcan_data_type_id_t type,
    float value
);
extern "C" void can_rx_cb(const CANMessage& message);
extern "C" void rc_pwm_cb(uint32_t width_us, uint32_t period_us);


void identification_cb(
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
            HARD_CURRENT_LIMIT_A * HARD_CURRENT_LIMIT_A) {
        g_fault = true;
        g_controller_mode = CONTROLLER_STOPPED;
    }
    if (g_controller_mode == CONTROLLER_STOPPED) {
        out_v_ab_v[0] = out_v_ab_v[1] = 0.0f;
    }

    g_vbus_v = vbus_v;
}


#pragma GCC optimize("O3")
void control_cb(
    float out_v_ab_v[2],
    const float last_v_ab_v[2],
    const float i_ab_a[2],
    float vbus_v
) {
    float v_dq_v[2], current_setpoint, speed_setpoint, readings[3], hfi_v[2],
          v_ab_v[2];
    struct motor_state_t motor_state;

    /* Update the state estimate with those values */
    g_estimator.update_state_estimate(i_ab_a, last_v_ab_v,
                                      g_current_controller_setpoint > 0.0f ?
                                      0.1f : -0.1f);
    g_estimator.get_state_estimate(motor_state);

    /*
    Handle stop condition -- set Vd, Vq to zero and hope the resulting hard
    stop doesn't break anything. Trigger at current limit.
    */
    if (i_ab_a[0] * i_ab_a[0] + i_ab_a[1] * i_ab_a[1] >
            HARD_CURRENT_LIMIT_A * HARD_CURRENT_LIMIT_A) {
        g_fault = true;
        g_controller_mode = CONTROLLER_STOPPED;
    }

    if (g_controller_mode == CONTROLLER_STOPPED) {
        out_v_ab_v[0] = out_v_ab_v[1] = v_dq_v[0] = v_dq_v[1] = 0.0f;
        current_setpoint = 0.0f;
        speed_setpoint = 0.0f;
        g_estimator.reset_state();
        g_current_controller.reset_state();
        g_speed_controller.reset_state();
        g_alignment_controller.reset_state();
        g_estimator_converged = false;
        readings[0] = readings[1] = readings[2] = 0.0f;
    } else if (g_estimator.is_converged()) {
        g_estimator_converged = true;
        /* Update the speed controller setpoint */
        if (g_controller_mode == CONTROLLER_TORQUE ||
                g_controller_mode == CONTROLLER_STOPPING) {
            speed_setpoint = motor_state.angular_velocity_rad_per_s;
            g_speed_controller_setpoint = speed_setpoint;
        } else {
            speed_setpoint = g_speed_controller_setpoint;
        }
        g_speed_controller.set_setpoint(speed_setpoint);

        /*
        Update the current controller with the output of the speed controller,
        except when
        */
        current_setpoint = g_speed_controller.update(motor_state);
        if (g_controller_mode == CONTROLLER_TORQUE ||
                g_controller_mode == CONTROLLER_STOPPING) {
            current_setpoint = g_current_controller_setpoint;
        } else {
            g_current_controller_setpoint = current_setpoint;
        }

        /* Update the stator current controller setpoint. */
        g_current_controller.set_setpoint(current_setpoint);

        /* Update the current controller and obtain new Vd and Vq */
        g_current_controller.update(v_dq_v,
                                    motor_state.i_dq_a,
                                    motor_state.angular_velocity_rad_per_s,
                                    vbus_v);

        /* Add high-frequency injection voltage */
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
        g_estimator_converged = false;
        /* Estimator is still aligning/converging */
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
    }

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
    g_estimator_hfi[2] = readings[2];
}


void systick_cb(void) {
    struct motor_state_t motor_state;
    union {
        uint8_t bytes[8];
        struct can_status_controller_t controller;
        struct can_status_measurement_t measurement;
        struct can_status_hfi_t hfi;
        struct can_status_estimator_t estimator;
    } message;
    UAVCANMessage out_message;
    CANMessage out_debug_message;
    struct uavcan_esc_state_t uavcan_state;
    float temp, v_dq_v[2], is_a, vs_v;

    motor_state = const_cast<struct motor_state_t&>(g_motor_state);
    v_dq_v[0] = g_v_dq_v[0];
    v_dq_v[1] = g_v_dq_v[1];

    uavcan_state.vbus_v = g_vbus_v;
    /*
    Try to work out how much current we're actually drawing from the supply,
    assuming 100% conversion efficiency.
    */
    is_a = __VSQRTF(motor_state.i_dq_a[0] * motor_state.i_dq_a[0] +
                    motor_state.i_dq_a[1] * motor_state.i_dq_a[1]);
    vs_v = __VSQRTF(v_dq_v[0] * v_dq_v[0] + v_dq_v[1] * v_dq_v[1]);
    uavcan_state.ibus_a = is_a * vs_v / g_vbus_v;
    uavcan_state.temperature_degc = hal_get_temperature_degc();
    uavcan_state.speed_rpm = motor_state.angular_velocity_rad_per_s *
        (60.0f / (float)M_PI) / (float)g_motor_params.num_poles;
    uavcan_state.power_pct = 100.0f * (is_a * vs_v) /
        (g_motor_params.max_current_a * g_motor_params.max_voltage_v);

    g_uavcan_server.tick(uavcan_state);

    /* Process UAVCAN transmissions */
    if (g_valid_can) {
        g_uavcan_server.process_tx(out_message);
        (void)hal_transmit_can_message(out_message);
    }

    /*
    Process debug CAN status transmissions whenever debug CAN is the active
    interface.
    */
    if (g_valid_can && g_last_can &&
            g_time - g_last_can < INTERFACE_TIMEOUT_MS) {
        if ((g_time % 20) == 0) {
            /* Send measurement status */
            message.measurement.node_id = g_can_node_id;
            message.measurement.temperature =
                (int8_t)hal_get_temperature_degc();
            temp = g_speed_controller_setpoint;
            temp *= (60.0f / (float)M_PI) / (float)g_motor_params.num_poles;
            message.measurement.rpm_setpoint = (int16_t)temp;
            temp = motor_state.angular_velocity_rad_per_s;
            temp *= (60.0f / (float)M_PI) / (float)g_motor_params.num_poles;
            message.measurement.rpm = (int16_t)temp;
            message.measurement.vbus = float16(g_vbus_v);
            out_debug_message.set_id(CAN_STATUS_MEASUREMENT);
            out_debug_message.set_data(sizeof(struct can_status_measurement_t),
                                       message.bytes);
            hal_transmit_can_message(out_debug_message);
        } else if ((g_time % 20) == 5) {
            /* Send HFI status */
            message.hfi.node_id = g_can_node_id;

            message.hfi.hfi_d = float16(g_estimator_hfi[0]);
            message.hfi.hfi_q = float16(g_estimator_hfi[1]);
            message.hfi.angle_rad = float16(motor_state.angle_rad *
                                            (0.5f / (float)M_PI));

            out_debug_message.set_id(CAN_STATUS_HFI);
            out_debug_message.set_data(sizeof(struct can_status_hfi_t),
                                       message.bytes);
            hal_transmit_can_message(out_debug_message);
        } else if ((g_time % 20) == 10) {
            /* Send controller status */
            message.controller.node_id = g_can_node_id;

            message.controller.id = float16(motor_state.i_dq_a[0]);
            message.controller.iq = float16(motor_state.i_dq_a[1]);
            message.controller.iq_setpoint =
                float16(g_current_controller_setpoint);

            out_debug_message.set_id(CAN_STATUS_CONTROLLER);
            out_debug_message.set_data(sizeof(struct can_status_controller_t),
                                       message.bytes);
            hal_transmit_can_message(out_debug_message);
        } else if ((g_time % 20) == 15) {
            /* Send controller status */
            message.estimator.node_id = g_can_node_id;

            //message.estimator.id = float16(motor_state.i_dq_a[0]);
            //message.estimator.iq = float16(motor_state.i_dq_a[1]);
            //message.estimator.iq_setpoint =
            //    float16(g_current_controller_setpoint);

            out_debug_message.set_id(CAN_STATUS_ESTIMATOR);
            out_debug_message.set_data(sizeof(struct can_status_estimator_t),
                                       message.bytes);
            hal_transmit_can_message(out_debug_message);
        }
    }

    g_time++;
    esc_assert(!g_fault);

    /* Motor shutdown logic -- stop when back EMF drops below 0.1 V */
    if (std::abs(motor_state.angular_velocity_rad_per_s) <
            0.01f / g_motor_params.phi_v_s_per_rad &&
            g_controller_mode == CONTROLLER_STOPPING) {
        g_controller_mode = CONTROLLER_STOPPED;
        g_speed_controller_setpoint = 0.0f;
        g_current_controller_setpoint = 0.0f;
    } else if (g_controller_mode == CONTROLLER_STOPPING) {
        g_current_controller_setpoint = 0.0f;
    } else if (g_controller_mode == CONTROLLER_SPEED ||
                g_controller_mode == CONTROLLER_TORQUE) {
        /*
        Stop gracefully if the current command mode doesn't provide an
        update in the required period
        */
        if ((g_input_source == INPUT_PWM && g_time - g_last_pwm > THROTTLE_TIMEOUT_MS) ||
                (g_input_source == INPUT_CAN && g_time - g_last_can > THROTTLE_TIMEOUT_MS) ||
                (g_input_source == INPUT_UAVCAN && g_time - g_last_uavcan > THROTTLE_TIMEOUT_MS)) {
            g_controller_mode = CONTROLLER_STOPPING;
            g_valid_pwm = 0;
        }
    }
}


bool can_restart_command_cb(void) {
    if (g_controller_mode == CONTROLLER_STOPPED) {
        hal_restart();
        g_fault = true;
        return true;
    } else {
        return false;
    }
}


bool can_write_flash_command_cb(void) {
    if (g_controller_mode == CONTROLLER_STOPPED) {
        g_configuration.write_params();
        return true;
    } else {
        return false;
    }
}


bool can_esc_command_cb(enum uavcan_data_type_id_t type, float value) {
    if (value > 0.0f) {
        if (type == UAVCAN_RAWCOMMAND) {
            g_controller_mode = CONTROLLER_TORQUE;
            g_current_controller_setpoint =
                g_motor_params.max_current_a * value * (1.0f / 8192.0f);
        } else if (type == UAVCAN_RPMCOMMAND) {
            g_controller_mode = CONTROLLER_SPEED;
            g_speed_controller_setpoint =
                value * (1.0f / 60.0f) * (float)g_motor_params.num_poles *
                (float)M_PI;
        }

        g_input_source = INPUT_UAVCAN;
        g_last_uavcan = g_time;
    }

    return true;
}


void can_rx_cb(const CANMessage& message) {
    union {
        uint8_t bytes[8];
        struct can_command_setpoint_t setpoint;
        struct can_command_config_t config;
        struct can_command_restart_t restart;
        struct can_status_config_t config_reply;
    } debug_message;
    UAVCANMessage m(message);
    CANMessage reply;
    float temp;
    uint8_t param_index;

    /* Enable transmission */
    g_valid_can = true;
    g_valid_pwm = 0;

    if (message.has_extended_id()) {
        /* UAVCAN message */
        g_uavcan_server.process_rx(m);
    } else {
        /* Debug CAN message */
        (void)message.get_data(debug_message.bytes);
        if (debug_message.bytes[0] == g_can_node_id) {
            switch (message.get_id()) {
                case CAN_COMMAND_SETPOINT:
                    /* Pass through to the UAVCAN ESC command controller */
                    switch (debug_message.setpoint.controller_mode) {
                        case CONTROLLER_TORQUE:
                            temp = (float)debug_message.setpoint.torque_setpoint;
                            if (temp < 0.0f || temp > 0.0f) {
                                g_controller_mode = CONTROLLER_TORQUE;
                                g_current_controller_setpoint =
                                    g_motor_params.max_current_a * temp *
                                    (1.0f / 8192.0f);
                                g_input_source = INPUT_CAN;
                                g_last_can = g_time;
                            }
                            break;
                        case CONTROLLER_SPEED:
                            temp = (float)debug_message.setpoint.rpm_setpoint;
                            if (temp < 0.0f || temp > 0.0f) {
                                g_controller_mode = CONTROLLER_SPEED;
                                temp *= (1.0f / 60.0f) *
                                        (float)g_motor_params.num_poles *
                                        (float)M_PI;
                                g_speed_controller_setpoint = temp;
                                g_input_source = INPUT_CAN;
                                g_last_can = g_time;
                            }
                            break;
                        default:
                            if (g_controller_mode == CONTROLLER_TORQUE ||
                                    g_controller_mode == CONTROLLER_SPEED) {
                                g_controller_mode = CONTROLLER_STOPPING;
                            }
                            break;
                    }
                    break;
                case CAN_COMMAND_CONFIG:
                    /*
                    Store the node ID to which this message was sent, so we
                    send the reply with that ID even if the node ID changes
                    (e.g. due to changing the uavcan_node_id parameter).
                    */
                    param_index = debug_message.config.param_index;
                    if (debug_message.config.set) {
                        g_configuration.set_param_value_by_index(
                            param_index, debug_message.config.param_value);
                    }
                    if (debug_message.config.save) {
                        can_write_flash_command_cb();
                    }
                    /* Reply with current parameter value */
                    temp = g_configuration.get_param_value_by_index(
                        param_index);
                    debug_message.config_reply.node_id = g_can_node_id;
                    debug_message.config_reply.param_index = param_index;
                    debug_message.config_reply.param_value = temp;
                    reply.set_data(sizeof(struct can_status_config_t),
                                   debug_message.bytes);
                    reply.set_id(CAN_STATUS_CONFIG);
                    /*
                    This is not thread-safe -- could result in packet loss
                    or corruption if this interrupt is called at the same time
                    as the timer task is transmitting.
                    */
                    hal_transmit_can_message(reply);
                    break;
                case CAN_COMMAND_RESTART:
                    if (debug_message.restart.magic == CAN_COMMAND_RESTART_MAGIC &&
                            g_controller_mode == CONTROLLER_STOPPED) {
                        hal_restart();
                    }
                    break;
            }
        }
    }
}


void rc_pwm_cb(uint32_t width_us, uint32_t period_us) {
    float setpoint;

    /* PWM arm/disarm logic */
    if (width_us > RC_PWM_MIN_WIDTH_US && width_us < RC_PWM_MAX_WIDTH_US &&
            period_us > RC_PWM_MIN_PERIOD_US &&
            period_us < RC_PWM_MAX_PERIOD_US && !g_fault) {
        g_valid_pwm++;

        if (g_valid_pwm >= RC_PWM_ARM_PULSES &&
                width_us > RC_PWM_ZERO_WIDTH_US) {
            g_input_source = INPUT_PWM;
            g_controller_mode = g_pwm_controller_mode;
            g_last_pwm = g_time;
        }

        /* Constrain the PWM signal to full scale */
        width_us = std::min((uint32_t)RC_PWM_FS_WIDTH_US, (uint32_t)width_us);

        /* Disable CAN if we have valid PWM */
        hal_disable_can_transmit();
        g_valid_can = false;
    } else if (g_valid_pwm > 0) {
        g_valid_pwm--;
    }

    /* If PWM control is armed, and the pulse is valid, update the setpoint */
    if (g_input_source == INPUT_PWM && !g_fault &&
            width_us >= RC_PWM_ZERO_WIDTH_US &&
            width_us <= RC_PWM_FS_WIDTH_US) {
        setpoint = (float)(width_us - RC_PWM_ZERO_WIDTH_US) /
                   (float)(RC_PWM_FS_WIDTH_US - RC_PWM_ZERO_WIDTH_US);
        setpoint = std::min(setpoint, 1.0f);

        if (g_controller_mode == CONTROLLER_SPEED) {
            /*
            PWM duty cycle controls thrust (proportional to square of RPM);
            maximum RPM is determined by configuration.
            */
            g_speed_controller_setpoint = g_motor_params.max_speed_rad_per_s *
                                          std::max(0.1f, __VSQRTF(setpoint));
        } else if (g_controller_mode == CONTROLLER_TORQUE) {
            /*
            PWM duty cycle controls torque (approximately proportional to
            thrust). Minimum throttle is
            */
            g_current_controller_setpoint =
                g_motor_params.max_current_a * std::max(0.1f, setpoint);
        }
    }
}


int __attribute__ ((externally_visible)) main(void) {
    struct control_params_t control_params;
    struct motor_params_t motor_params;
    uint32_t i;
    float i_samples[4], v_samples[4], rs_r, ls_h, limit_speed;

    hal_reset();
    hal_set_pwm_state(HAL_PWM_STATE_LOW);

    /* Read parameters from flash */
    g_configuration.read_motor_params(motor_params);
    g_configuration.read_control_params(control_params);

    /* Estimate motor parameters */
    g_parameter_estimator_done = false;
    g_parameter_estimator.start_estimation(hal_control_t_s);
    hal_set_high_frequency_callback(identification_cb);

    hal_set_pwm_state(HAL_PWM_STATE_RUNNING);
    g_controller_mode = CONTROLLER_IDENTIFYING;

    while (!g_parameter_estimator_done &&
            g_controller_mode == CONTROLLER_IDENTIFYING);

    hal_set_pwm_state(HAL_PWM_STATE_LOW);
    g_controller_mode = CONTROLLER_STOPPED;

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
    g_configuration.set_param_value_by_index(PARAM_MOTOR_RS, rs_r);
    g_configuration.set_param_value_by_index(PARAM_MOTOR_LS, ls_h);

    /* Get the node ID */
    g_can_node_id = (uint8_t)
        g_configuration.get_param_value_by_index(PARAM_UAVCAN_NODE_ID);

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

    g_motor_params.rs_r = motor_params.rs_r;
    g_motor_params.ls_h = motor_params.ls_h;
    g_motor_params.phi_v_s_per_rad = motor_params.phi_v_s_per_rad;
    g_motor_params.max_voltage_v = motor_params.max_voltage_v;
    g_motor_params.max_current_a = motor_params.max_current_a;
    g_motor_params.max_speed_rad_per_s = motor_params.max_speed_rad_per_s;
    g_motor_params.num_poles = motor_params.num_poles;

    /* Start PWM */
    hal_set_pwm_state(HAL_PWM_STATE_RUNNING);

    /* Register CAN callbacks */
    g_uavcan_server.set_flash_save_callback(can_write_flash_command_cb);
    g_uavcan_server.set_restart_request_callback(can_restart_command_cb);
    g_uavcan_server.set_esc_command_callback(can_esc_command_cb);

    /*
    After starting the ISR tasks we are no longer able to access g_estimator,
    g_current_controller and g_speed_controller, since they're updated from
    the ISRs and not declared volatile.
    */
    hal_set_can_receive_callback(can_rx_cb);
    hal_set_high_frequency_callback(control_cb);
    hal_set_low_frequency_callback(systick_cb);
    hal_set_rc_pwm_callback(rc_pwm_cb);

    for (volatile bool cond = true; cond;);

    return 0;
}
