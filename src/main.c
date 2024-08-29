// Copyright 2019 - 2022 Mitch Lustig
// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "balance_filter.h"

#include "vesc_c_if.h"

#include "atr.h"
#include "charging.h"
#include "footpad_sensor.h"
#include "lcm.h"
#include "leds.h"
#include "motor_data.h"
#include "state.h"
#include "torque_tilt.h"
#include "utils.h"

#include "conf/buffer.h"
#include "conf/conf_general.h"
#include "conf/confparser.h"
#include "conf/confxml.h"
#include "conf/datatypes.h"

#include "konami.h"

#include <math.h>
#include <string.h>

HEADER

typedef enum {
    BEEP_NONE = 0,
    BEEP_LV = 1,
    BEEP_HV = 2,
    BEEP_TEMPFET = 3,
    BEEP_TEMPMOT = 4,
    BEEP_CURRENT = 5,
    BEEP_DUTY = 6,
    BEEP_SENSORS = 7,
    BEEP_LOWBATT = 8,
    BEEP_IDLE = 9,
    BEEP_ERROR = 10
} BeepReason;

static const FootpadSensorState flywheel_konami_sequence[] = {
    FS_LEFT, FS_NONE, FS_RIGHT, FS_NONE, FS_LEFT, FS_NONE, FS_RIGHT
};

// This is all persistent state of the application, which will be allocated in init. It
// is put here because variables can only be read-only when this program is loaded
// in flash without virtual memory in RAM (as all RAM already is dedicated to the
// main firmware and managed from there). This is probably the main limitation of
// loading applications in runtime, but it is not too bad to work around.
typedef struct {
    lib_thread main_thread;
    lib_thread led_thread;

    RefloatConfig float_conf;

    // Firmware version, passed in from Lisp
    int fw_version_major, fw_version_minor, fw_version_beta;

    MotorData motor;
    TorqueTilt torque_tilt;
    ATR atr;

    // Beeper
    int beep_num_left;
    int beep_duration;
    int beep_countdown;
    int beep_reason;
    bool beeper_enabled;

    Leds leds;

    // Lights Control Module - external lights control
    LcmData lcm;

    Charging charging;

    // Config values
    uint32_t loop_time_us;
    unsigned int start_counter_clicks, start_counter_clicks_max;
    float startup_pitch_trickmargin, startup_pitch_tolerance;
    float startup_step_size;
    float tiltback_duty_step_size, tiltback_hv_step_size, tiltback_lv_step_size,
        tiltback_return_step_size;
    float turntilt_step_size;
    float tiltback_variable, tiltback_variable_max_erpm, noseangling_step_size,
        inputtilt_ramped_step_size, inputtilt_step_size;
    float mc_max_temp_fet, mc_max_temp_mot;
    float mc_current_max, mc_current_min;
    float surge_angle, surge_angle2, surge_angle3, surge_adder;
    bool surge_enable;
    bool duty_beeping;

    // IMU data for the balancing filter
    BalanceFilterData balance_filter;

    // Runtime values read from elsewhere
    float pitch, roll;
    float balance_pitch;
    float gyro[3];

    float throttle_val;
    float max_duty_with_margin;

    FootpadSensor footpad_sensor;

    // Feature: Turntilt
    float last_yaw_angle, yaw_angle, abs_yaw_change, last_yaw_change, yaw_change, yaw_aggregate;
    float turntilt_boost_per_erpm, yaw_aggregate_target;

    // Rumtime state values
    State state;

    float proportional;
    float integral;
    float rate_p;
    float pid_value;

    float setpoint, setpoint_target, setpoint_target_interpolated;
    float applied_booster_current;
    float noseangling_interpolated, inputtilt_interpolated;
    float turntilt_target, turntilt_interpolated;
    float current_time;
    float disengage_timer, nag_timer;
    float idle_voltage;
    float fault_angle_pitch_timer, fault_angle_roll_timer, fault_switch_timer,
        fault_switch_half_timer;
    float motor_timeout_s;
    float brake_timeout;
    float wheelslip_timer, tb_highvoltage_timer;
    float switch_warn_beep_erpm;
    bool traction_control;

    // PID Brake Scaling
    float kp_brake_scale;  // Used for brakes when riding forwards, and accel when riding backwards
    float kp2_brake_scale;
    float kp_accel_scale;  // Used for accel when riding forwards, and brakes when riding backwards
    float kp2_accel_scale;

    // Darkride aka upside down mode:
    bool is_upside_down_started;  // dark ride has been engaged
    bool enable_upside_down;  // dark ride mode is enabled (10 seconds after fault)
    float delay_upside_down_fault;
    float darkride_setpoint_correction;

    // Feature: Flywheel
    bool flywheel_abort;
    float flywheel_pitch_offset, flywheel_roll_offset;

    // Feature: Reverse Stop
    float reverse_stop_step_size, reverse_tolerance, reverse_total_erpm;
    float reverse_timer;

    // Feature: Soft Start
    float softstart_pid_limit, softstart_ramp_step_size;

    // Odometer
    float odo_timer;
    int odometer_dirty;
    uint64_t odometer;

    // Feature: RC Move (control via app while idle)
    int rc_steps;
    int rc_counter;
    float rc_current_target;
    float rc_current;

    Konami flywheel_konami;
} data;

static void brake(data *d);
static void set_current(data *d, float current);
static void flywheel_stop(data *d);
static void cmd_flywheel_toggle(data *d, unsigned char *cfg, int len);

const VESC_PIN beeper_pin = VESC_PIN_PPM;

#define EXT_BEEPER_ON() VESC_IF->io_write(beeper_pin, 1)
#define EXT_BEEPER_OFF() VESC_IF->io_write(beeper_pin, 0)

void beeper_init() {
    VESC_IF->io_set_mode(beeper_pin, VESC_PIN_MODE_OUTPUT);
}

void beeper_update(data *d) {
    if (d->beeper_enabled && (d->beep_num_left > 0)) {
        d->beep_countdown--;
        if (d->beep_countdown <= 0) {
            d->beep_countdown = d->beep_duration;
            d->beep_num_left--;
            if (d->beep_num_left & 0x1) {
                EXT_BEEPER_ON();
            } else {
                EXT_BEEPER_OFF();
            }
        }
    }
}

void beeper_enable(data *d, bool enable) {
    d->beeper_enabled = enable;
    if (!enable) {
        EXT_BEEPER_OFF();
    }
}

void beep_alert(data *d, int num_beeps, bool longbeep) {
    if (!d->beeper_enabled) {
        return;
    }
    if (d->beep_num_left == 0) {
        d->beep_num_left = num_beeps * 2 + 1;
        d->beep_duration = longbeep ? 300 : 80;
        d->beep_countdown = d->beep_duration;
    }
}

void beep_off(data *d, bool force) {
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_OFF();
    }
}

void beep_on(data *d, bool force) {
    if (!d->beeper_enabled) {
        return;
    }
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_ON();
    }
}

static void reconfigure(data *d) {
    motor_data_configure(&d->motor, d->float_conf.atr_filter / d->float_conf.hertz);
    balance_filter_configure(&d->balance_filter, &d->float_conf);
    torque_tilt_configure(&d->torque_tilt, &d->float_conf);
    atr_configure(&d->atr, &d->float_conf);
}

static void configure(data *d) {
    state_init(&d->state, d->float_conf.disabled);

    lcm_configure(&d->lcm, &d->float_conf.leds);

    // This timer is used to determine how long the board has been disengaged / idle
    d->disengage_timer = d->current_time;

    // Loop time in microseconds
    d->loop_time_us = 1e6 / d->float_conf.hertz;

    // Loop time in seconds times 20 for a nice long grace period
    d->motor_timeout_s = 20.0f / d->float_conf.hertz;

    d->startup_step_size = d->float_conf.startup_speed / d->float_conf.hertz;
    d->tiltback_duty_step_size = d->float_conf.tiltback_duty_speed / d->float_conf.hertz;
    d->tiltback_hv_step_size = d->float_conf.tiltback_hv_speed / d->float_conf.hertz;
    d->tiltback_lv_step_size = d->float_conf.tiltback_lv_speed / d->float_conf.hertz;
    d->tiltback_return_step_size = d->float_conf.tiltback_return_speed / d->float_conf.hertz;
    d->turntilt_step_size = d->float_conf.turntilt_speed / d->float_conf.hertz;
    d->noseangling_step_size = d->float_conf.noseangling_speed / d->float_conf.hertz;
    d->inputtilt_step_size = d->float_conf.inputtilt_speed / d->float_conf.hertz;

    d->surge_angle = d->float_conf.surge_angle;
    d->surge_angle2 = d->float_conf.surge_angle * 2;
    d->surge_angle3 = d->float_conf.surge_angle * 3;
    d->surge_enable = d->surge_angle > 0;

    // Feature: Stealthy start vs normal start (noticeable click when engaging) - 0-20A
    d->start_counter_clicks_max = 3;
    // Feature: Soft Start
    d->softstart_ramp_step_size = (float) 100 / d->float_conf.hertz;
    // Feature: Dirty Landings
    d->startup_pitch_trickmargin = d->float_conf.startup_dirtylandings_enabled ? 10 : 0;

    // Backwards compatibility hack:
    // If mahony kp from the firmware internal filter is higher than 1, it's
    // the old setup with it being the main balancing filter. In that case, set
    // the kp and acc confidence decay to hardcoded defaults of the former true
    // pitch filter, to preserve the behavior of the old setup in the new one.
    // (Though Mahony KP 0.4 instead of 0.2 is used, as it seems to work better)
    if (VESC_IF->get_cfg_float(CFG_PARAM_IMU_mahony_kp) > 1) {
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_kp, 0.4);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_ki, 0);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_accel_confidence_decay, 0.1);
    }

    d->mc_max_temp_fet = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_fet_start) - 3;
    d->mc_max_temp_mot = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_motor_start) - 3;

    d->mc_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    // min current is a positive value here!
    d->mc_current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));

    d->max_duty_with_margin = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty) - 0.1;

    // Feature: Reverse Stop
    d->reverse_tolerance = 20000;
    d->reverse_stop_step_size = 100.0 / d->float_conf.hertz;

    // Feature: Turntilt
    d->yaw_aggregate_target = fmaxf(50, d->float_conf.turntilt_yaw_aggregate);
    d->turntilt_boost_per_erpm = (float) d->float_conf.turntilt_erpm_boost / 100.0 /
        (float) d->float_conf.turntilt_erpm_boost_end;

    // Feature: Darkride
    d->enable_upside_down = false;
    d->darkride_setpoint_correction = d->float_conf.dark_pitch_offset;

    // Feature: Flywheel
    d->flywheel_abort = false;

    // Allows smoothing of Remote Tilt
    d->inputtilt_ramped_step_size = 0;

    // Speed above which to warn users about an impending full switch fault
    d->switch_warn_beep_erpm = d->float_conf.is_footbeep_enabled ? 2000 : 100000;

    // Variable nose angle adjustment / tiltback (setting is per 1000erpm, convert to per erpm)
    d->tiltback_variable = d->float_conf.tiltback_variable / 1000;
    if (d->tiltback_variable > 0) {
        d->tiltback_variable_max_erpm =
            fabsf(d->float_conf.tiltback_variable_max / d->tiltback_variable);
    } else {
        d->tiltback_variable_max_erpm = 100000;
    }

    d->beeper_enabled = d->float_conf.is_beeper_enabled;

    konami_init(&d->flywheel_konami, flywheel_konami_sequence, sizeof(flywheel_konami_sequence));

    reconfigure(d);

    if (d->state.state == STATE_DISABLED) {
        beep_alert(d, 3, false);
    } else {
        beep_alert(d, 1, false);
    }
}

static void reset_vars(data *d) {
    motor_data_reset(&d->motor);
    atr_reset(&d->atr);
    torque_tilt_reset(&d->torque_tilt);

    // Set values for startup
    d->setpoint = d->balance_pitch;
    d->setpoint_target_interpolated = d->balance_pitch;
    d->setpoint_target = 0;
    d->applied_booster_current = 0;
    d->noseangling_interpolated = 0;
    d->inputtilt_interpolated = 0;
    d->turntilt_target = 0;
    d->turntilt_interpolated = 0;
    d->brake_timeout = 0;
    d->traction_control = false;
    d->pid_value = 0;
    d->rate_p = 0;
    d->integral = 0;
    d->softstart_pid_limit = 0;
    d->startup_pitch_tolerance = d->float_conf.startup_pitch_tolerance;
    d->surge_adder = 0;

    // PID Brake Scaling
    d->kp_brake_scale = 1.0;
    d->kp2_brake_scale = 1.0;
    d->kp_accel_scale = 1.0;
    d->kp2_accel_scale = 1.0;

    // Turntilt:
    d->last_yaw_angle = 0;
    d->yaw_aggregate = 0;

    // Feature: click on start
    d->start_counter_clicks = d->start_counter_clicks_max;

    // RC Move:
    d->rc_steps = 0;
    d->rc_current = 0;

    state_engage(&d->state);
}

/**
 * check_odometer: see if we need to write back the odometer during fault state
 */
static void check_odometer(data *d) {
    // Make odometer persistent if we've gone 200m or more
    if (d->odometer_dirty > 0) {
        float stored_odo = VESC_IF->mc_get_odometer();
        if ((stored_odo > d->odometer + 200) || (stored_odo < d->odometer - 10000)) {
            if (d->odometer_dirty == 1) {
                // Wait 10 seconds before writing to avoid writing if immediately continuing to ride
                d->odo_timer = d->current_time;
                d->odometer_dirty++;
            } else if ((d->current_time - d->odo_timer) > 10) {
                VESC_IF->store_backup_data();
                d->odometer = VESC_IF->mc_get_odometer();
                d->odometer_dirty = 0;
            }
        }
    }
}

/**
 *  do_rc_move: perform motor movement while board is idle
 */
static void do_rc_move(data *d) {
    if (d->rc_steps > 0) {
        d->rc_current = d->rc_current * 0.95 + d->rc_current_target * 0.05;
        if (d->motor.abs_erpm > 800) {
            d->rc_current = 0;
        }
        set_current(d, d->rc_current);
        d->rc_steps--;
        d->rc_counter++;
        if ((d->rc_counter == 500) && (d->rc_current_target > 2)) {
            d->rc_current_target /= 2;
        }
    } else {
        d->rc_counter = 0;

        // Throttle must be greater than 2% (Help mitigate lingering throttle)
        if ((d->float_conf.remote_throttle_current_max > 0) &&
            (d->current_time - d->disengage_timer > d->float_conf.remote_throttle_grace_period) &&
            (fabsf(d->throttle_val) > 0.02)) {
            float servo_val = d->throttle_val;
            servo_val *= (d->float_conf.inputtilt_invert_throttle ? -1.0 : 1.0);
            d->rc_current = d->rc_current * 0.95 +
                (d->float_conf.remote_throttle_current_max * servo_val) * 0.05;
            set_current(d, d->rc_current);
        } else {
            d->rc_current = 0;
            // Disable output
            brake(d);
        }
    }
}

static float get_setpoint_adjustment_step_size(data *d) {
    switch (d->state.sat) {
    case (SAT_NONE):
        return d->tiltback_return_step_size;
    case (SAT_CENTERING):
        return d->startup_step_size;
    case (SAT_REVERSESTOP):
        return d->reverse_stop_step_size;
    case (SAT_PB_DUTY):
        return d->tiltback_duty_step_size;
    case (SAT_PB_HIGH_VOLTAGE):
    case (SAT_PB_TEMPERATURE):
        return d->tiltback_hv_step_size;
    case (SAT_PB_LOW_VOLTAGE):
        return d->tiltback_lv_step_size;
    default:
        return 0;
    }
}

bool is_engaged(const data *d) {
    if (d->footpad_sensor.state == FS_BOTH) {
        return true;
    }

    if (d->footpad_sensor.state == FS_LEFT || d->footpad_sensor.state == FS_RIGHT) {
        // 5 seconds after stopping we allow starting with a single sensor (e.g. for jump starts)
        bool is_simple_start =
            d->float_conf.startup_simplestart_enabled && (d->current_time - d->disengage_timer > 5);

        if (d->float_conf.fault_is_dual_switch || is_simple_start) {
            return true;
        }
    }

    // Keep the board engaged in flywheel mode
    if (d->state.mode == MODE_FLYWHEEL) {
        return true;
    }

    return false;
}

// Fault checking order does not really matter. From a UX perspective, switch should be before
// angle.
static bool check_faults(data *d) {
    // Aggressive reverse stop in case the board runs off when upside down
    if (d->state.darkride) {
        if (d->motor.erpm > 1000) {
            // erpms are also reversed when upside down!
            if (((d->current_time - d->fault_switch_timer) * 1000 > 100) || d->motor.erpm > 2000 ||
                (d->state.wheelslip && (d->current_time - d->delay_upside_down_fault > 1) &&
                 ((d->current_time - d->fault_switch_timer) * 1000 > 30))) {

                // Trigger Reverse Stop when board is going reverse and
                // going > 2mph for more than 100ms
                // going > 4mph
                // detecting wheelslip (aka excorcist wiggle) after the first second
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
        } else {
            d->fault_switch_timer = d->current_time;
            if (d->motor.erpm > 300) {
                // erpms are also reversed when upside down!
                if ((d->current_time - d->fault_angle_roll_timer) * 1000 > 500) {
                    state_stop(&d->state, STOP_REVERSE_STOP);
                    return true;
                }
            } else {
                d->fault_angle_roll_timer = d->current_time;
            }
        }
        if (is_engaged(d)) {
            // allow turning it off by engaging foot sensors
            state_stop(&d->state, STOP_SWITCH_HALF);
            return true;
        }
    } else {
        bool disable_switch_faults = d->float_conf.fault_moving_fault_disabled &&
            // Rolling forward (not backwards!)
            d->motor.erpm > (d->float_conf.fault_adc_half_erpm * 2) &&
            // Not tipped over
            fabsf(d->roll) < 40;

        // Check switch
        // Switch fully open
        if (d->footpad_sensor.state == FS_NONE && d->state.mode != MODE_FLYWHEEL) {
            if (!disable_switch_faults) {
                if ((1000.0 * (d->current_time - d->fault_switch_timer)) >
                    d->float_conf.fault_delay_switch_full) {
                    state_stop(&d->state, STOP_SWITCH_FULL);
                    return true;
                }
                // low speed (below 6 x half-fault threshold speed):
                else if ((d->motor.abs_erpm < d->float_conf.fault_adc_half_erpm * 6) &&
                         (1000.0 * (d->current_time - d->fault_switch_timer) >
                          d->float_conf.fault_delay_switch_half)) {
                    state_stop(&d->state, STOP_SWITCH_FULL);
                    return true;
                }
            }

            if (d->motor.abs_erpm < 200 && fabsf(d->pitch) > 14 &&
                fabsf(d->inputtilt_interpolated) < 30 && sign(d->pitch) == d->motor.erpm_sign) {
                state_stop(&d->state, STOP_QUICKSTOP);
                return true;
            }
        } else {
            d->fault_switch_timer = d->current_time;
        }

        // Feature: Reverse-Stop
        if (d->state.sat == SAT_REVERSESTOP) {
            //  Taking your foot off entirely while reversing? Ignore delays
            if (d->footpad_sensor.state == FS_NONE) {
                state_stop(&d->state, STOP_SWITCH_FULL);
                return true;
            }
            if (fabsf(d->pitch) > 18) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            // Above 10 degrees for a full second? Switch it off
            if (fabsf(d->pitch) > 10 && d->current_time - d->reverse_timer > 1) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            // Above 5 degrees for 2 seconds? Switch it off
            if (fabsf(d->pitch) > 5 && d->current_time - d->reverse_timer > 2) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            if (fabsf(d->reverse_total_erpm) > d->reverse_tolerance * 10) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            if (fabsf(d->pitch) < 5) {
                d->reverse_timer = d->current_time;
            }
        }

        // Switch partially open and stopped
        if (!d->float_conf.fault_is_dual_switch) {
            if (!is_engaged(d) && d->motor.abs_erpm < d->float_conf.fault_adc_half_erpm) {
                if ((1000.0 * (d->current_time - d->fault_switch_half_timer)) >
                    d->float_conf.fault_delay_switch_half) {
                    state_stop(&d->state, STOP_SWITCH_HALF);
                    return true;
                }
            } else {
                d->fault_switch_half_timer = d->current_time;
            }
        }

        // Check roll angle
        if (fabsf(d->roll) > d->float_conf.fault_roll) {
            if ((1000.0 * (d->current_time - d->fault_angle_roll_timer)) >
                d->float_conf.fault_delay_roll) {
                state_stop(&d->state, STOP_ROLL);
                return true;
            }
        } else {
            d->fault_angle_roll_timer = d->current_time;

            if (d->float_conf.fault_darkride_enabled) {
                if (fabsf(d->roll) > 100 && fabsf(d->roll) < 135) {
                    state_stop(&d->state, STOP_ROLL);
                    return true;
                }
            }
        }

        if (d->state.mode == MODE_FLYWHEEL && d->footpad_sensor.state == FS_BOTH) {
            state_stop(&d->state, STOP_SWITCH_HALF);
            d->flywheel_abort = true;
            return true;
        }
    }

    // Check pitch angle
    if (fabsf(d->pitch) > d->float_conf.fault_pitch && fabsf(d->inputtilt_interpolated) < 30) {
        if ((1000.0 * (d->current_time - d->fault_angle_pitch_timer)) >
            d->float_conf.fault_delay_pitch) {
            state_stop(&d->state, STOP_PITCH);
            return true;
        }
    } else {
        d->fault_angle_pitch_timer = d->current_time;
    }

    return false;
}

static void calculate_setpoint_target(data *d) {
    float input_voltage = VESC_IF->mc_get_input_voltage_filtered();

    if (input_voltage < d->float_conf.tiltback_hv) {
        d->tb_highvoltage_timer = d->current_time;
    }

    if (d->state.sat == SAT_CENTERING) {
        if (d->setpoint_target_interpolated == d->setpoint_target) {
            d->state.sat = SAT_NONE;
        }
    } else if (d->state.sat == SAT_REVERSESTOP) {
        // accumalete erpms:
        d->reverse_total_erpm += d->motor.erpm;
        if (fabsf(d->reverse_total_erpm) > d->reverse_tolerance) {
            // tilt down by 10 degrees after exceeding aggregate erpm
            d->setpoint_target =
                10 * (fabsf(d->reverse_total_erpm) - d->reverse_tolerance) * 0.000008;
        } else {
            if (fabsf(d->reverse_total_erpm) <= d->reverse_tolerance * 0.5) {
                if (d->motor.erpm >= 0) {
                    d->state.sat = SAT_NONE;
                    d->reverse_total_erpm = 0;
                    d->setpoint_target = 0;
                    d->integral = 0;
                }
            }
        }
    } else if (d->state.mode != MODE_FLYWHEEL &&
               // not normal, either wheelslip or wheel getting stuck
               fabsf(d->motor.acceleration) > 15 &&
               sign(d->motor.acceleration) == d->motor.erpm_sign && d->motor.duty_cycle > 0.3 &&
               // acceleration can jump a lot at very low speeds
               d->motor.abs_erpm > 2000) {
        d->state.wheelslip = true;
        d->state.sat = SAT_NONE;
        d->wheelslip_timer = d->current_time;
        if (d->state.darkride) {
            d->traction_control = true;
        }
    } else if (d->state.wheelslip) {
        if (fabsf(d->motor.acceleration) < 10) {
            // acceleration is slowing down, traction control seems to have worked
            d->traction_control = false;
        }
        // Remain in wheelslip state for at least 500ms to avoid any overreactions
        if (d->motor.duty_cycle > d->max_duty_with_margin) {
            d->wheelslip_timer = d->current_time;
        } else if (d->current_time - d->wheelslip_timer > 0.2) {
            if (d->motor.duty_cycle < 0.7) {
                // Leave wheelslip state only if duty < 70%
                d->traction_control = false;
                d->state.wheelslip = false;
            }
        }
        if (d->float_conf.fault_reversestop_enabled && (d->motor.erpm < 0)) {
            // the 500ms wheelslip time can cause us to blow past the reverse stop condition!
            d->state.sat = SAT_REVERSESTOP;
            d->reverse_timer = d->current_time;
            d->reverse_total_erpm = 0;
        }
    } else if (d->motor.duty_cycle > d->float_conf.tiltback_duty) {
        if (d->motor.erpm > 0) {
            d->setpoint_target = d->float_conf.tiltback_duty_angle;
        } else {
            d->setpoint_target = -d->float_conf.tiltback_duty_angle;
        }

        // FLYWHEEL relies on the duty pushback mechanism, but we don't
        // want to show the pushback alert.
        //
        // TODO: Move FLYWHEEL checks out of this function and instead
        // implement a custom simple control loop for it, which won't need most
        // of the normal balancing features.
        if (d->state.mode != MODE_FLYWHEEL) {
            d->state.sat = SAT_PB_DUTY;
        }
    } else if (d->motor.duty_cycle > 0.05 && input_voltage > d->float_conf.tiltback_hv) {
        d->beep_reason = BEEP_HV;
        beep_alert(d, 3, false);
        if (((d->current_time - d->tb_highvoltage_timer) > .5) ||
            (input_voltage > d->float_conf.tiltback_hv + 1)) {
            // 500ms have passed or voltage is another volt higher, time for some tiltback
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->float_conf.tiltback_hv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_hv_angle;
            }

            d->state.sat = SAT_PB_HIGH_VOLTAGE;
        } else {
            // The rider has 500ms to react to the triple-beep, or maybe it was just a short spike
            d->state.sat = SAT_NONE;
        }
    } else if (VESC_IF->mc_temp_fet_filtered() > d->mc_max_temp_fet) {
        // Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
        beep_alert(d, 3, true);
        d->beep_reason = BEEP_TEMPFET;
        if (VESC_IF->mc_temp_fet_filtered() > (d->mc_max_temp_fet + 1)) {
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->float_conf.tiltback_lv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_lv_angle;
            }
            d->state.sat = SAT_PB_TEMPERATURE;
        } else {
            // The rider has 1 degree Celsius left before we start tilting back
            d->state.sat = SAT_NONE;
        }
    } else if (VESC_IF->mc_temp_motor_filtered() > d->mc_max_temp_mot) {
        // Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
        beep_alert(d, 3, true);
        d->beep_reason = BEEP_TEMPMOT;
        if (VESC_IF->mc_temp_motor_filtered() > (d->mc_max_temp_mot + 1)) {
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->float_conf.tiltback_lv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_lv_angle;
            }
            d->state.sat = SAT_PB_TEMPERATURE;
        } else {
            // The rider has 1 degree Celsius left before we start tilting back
            d->state.sat = SAT_NONE;
        }
    } else if (d->motor.duty_cycle > 0.05 && input_voltage < d->float_conf.tiltback_lv) {
        beep_alert(d, 3, false);
        d->beep_reason = BEEP_LV;
        float abs_motor_current = fabsf(d->motor.current);
        float vdelta = d->float_conf.tiltback_lv - input_voltage;
        float ratio = vdelta * 20 / abs_motor_current;
        // When to do LV tiltback:
        // a) we're 2V below lv threshold
        // b) motor current is small (we cannot assume vsag)
        // c) we have more than 20A per Volt of difference (we tolerate some amount of vsag)
        if ((vdelta > 2) || (abs_motor_current < 5) || (ratio > 1)) {
            if (d->motor.erpm > 0) {
                d->setpoint_target = d->float_conf.tiltback_lv_angle;
            } else {
                d->setpoint_target = -d->float_conf.tiltback_lv_angle;
            }

            d->state.sat = SAT_PB_LOW_VOLTAGE;
        } else {
            d->state.sat = SAT_NONE;
            d->setpoint_target = 0;
        }
    } else {
        // Normal running
        if (d->float_conf.fault_reversestop_enabled && d->motor.erpm < -200 && !d->state.darkride) {
            d->state.sat = SAT_REVERSESTOP;
            d->reverse_timer = d->current_time;
            d->reverse_total_erpm = 0;
        } else {
            d->state.sat = SAT_NONE;
        }
        d->setpoint_target = 0;
    }

    if (d->state.wheelslip && d->motor.duty_cycle > d->max_duty_with_margin) {
        d->setpoint_target = 0;
    }
    if (d->state.darkride) {
        if (!d->is_upside_down_started) {
            // right after flipping when first engaging dark ride we add a 1 second grace period
            // before aggressively checking for board wiggle (based on acceleration)
            d->is_upside_down_started = true;
            d->delay_upside_down_fault = d->current_time;
        }
    }

    if (d->state.mode != MODE_FLYWHEEL) {
        if (d->state.sat == SAT_PB_DUTY) {
            if (d->float_conf.is_dutybeep_enabled || (d->float_conf.tiltback_duty_angle == 0)) {
                beep_on(d, true);
                d->beep_reason = BEEP_DUTY;
                d->duty_beeping = true;
            }
        } else {
            if (d->duty_beeping) {
                beep_off(d, false);
            }
        }
    }
}

static void calculate_setpoint_interpolated(data *d) {
    if (d->setpoint_target_interpolated != d->setpoint_target) {
        rate_limitf(
            &d->setpoint_target_interpolated,
            d->setpoint_target,
            get_setpoint_adjustment_step_size(d)
        );
    }
}

static void add_surge(data *d) {
    if (d->surge_enable) {
        float surge_now = 0;

        if (d->motor.duty_smooth > d->float_conf.surge_duty_start + 0.04) {
            surge_now = d->surge_angle3;
            beep_alert(d, 3, 1);
        } else if (d->motor.duty_smooth > d->float_conf.surge_duty_start + 0.02) {
            surge_now = d->surge_angle2;
            beep_alert(d, 2, 1);
        } else if (d->motor.duty_smooth > d->float_conf.surge_duty_start) {
            surge_now = d->surge_angle;
            beep_alert(d, 1, 1);
        }
        if (surge_now >= d->surge_adder) {
            // kick in instantly
            d->surge_adder = surge_now;
        } else {
            // release less harshly
            d->surge_adder = d->surge_adder * 0.98 + surge_now * 0.02;
        }

        // Add surge angle to setpoint
        if (d->motor.erpm > 0) {
            d->setpoint += d->surge_adder;
        } else {
            d->setpoint -= d->surge_adder;
        }
    }
}

static void apply_noseangling(data *d) {
    // Nose angle adjustment, add variable then constant tiltback
    float noseangling_target = 0;

    // Variable Tiltback looks at ERPM from the reference point of the set minimum ERPM
    float variable_erpm = fmaxf(0, d->motor.abs_erpm - d->float_conf.tiltback_variable_erpm);
    if (variable_erpm > d->tiltback_variable_max_erpm) {
        noseangling_target = d->float_conf.tiltback_variable_max * d->motor.erpm_sign;
    } else {
        noseangling_target = d->tiltback_variable * variable_erpm * d->motor.erpm_sign *
            sign(d->float_conf.tiltback_variable_max);
    }

    if (d->motor.abs_erpm > d->float_conf.tiltback_constant_erpm) {
        noseangling_target += d->float_conf.tiltback_constant * d->motor.erpm_sign;
    }

    rate_limitf(&d->noseangling_interpolated, noseangling_target, d->noseangling_step_size);

    d->setpoint += d->noseangling_interpolated;
}

static void apply_inputtilt(data *d) {
    float input_tiltback_target;

    // Scale by Max Angle
    input_tiltback_target = d->throttle_val * d->float_conf.inputtilt_angle_limit;

    // Invert for Darkride
    input_tiltback_target *= (d->state.darkride ? -1.0 : 1.0);

    float input_tiltback_target_diff = input_tiltback_target - d->inputtilt_interpolated;

    // Smoothen changes in tilt angle by ramping the step size
    if (d->float_conf.inputtilt_smoothing_factor > 0) {
        float smoothing_factor = 0.02;
        for (int i = 1; i < d->float_conf.inputtilt_smoothing_factor; i++) {
            smoothing_factor /= 2;
        }

        // Sets the angle away from Target that step size begins ramping down
        float smooth_center_window = 1.5 + (0.5 * d->float_conf.inputtilt_smoothing_factor);

        // Within X degrees of Target Angle, start ramping down step size
        if (fabsf(input_tiltback_target_diff) < smooth_center_window) {
            // Target step size is reduced the closer to center you are (needed for smoothly
            // transitioning away from center)
            d->inputtilt_ramped_step_size =
                (smoothing_factor * d->inputtilt_step_size * (input_tiltback_target_diff / 2)) +
                ((1 - smoothing_factor) * d->inputtilt_ramped_step_size);
            // Linearly ramped down step size is provided as minimum to prevent overshoot
            float centering_step_size =
                fminf(
                    fabsf(d->inputtilt_ramped_step_size),
                    fabsf(input_tiltback_target_diff / 2) * d->inputtilt_step_size
                ) *
                sign(input_tiltback_target_diff);
            if (fabsf(input_tiltback_target_diff) < fabsf(centering_step_size)) {
                d->inputtilt_interpolated = input_tiltback_target;
            } else {
                d->inputtilt_interpolated += centering_step_size;
            }
        } else {
            // Ramp up step size until the configured tilt speed is reached
            d->inputtilt_ramped_step_size =
                (smoothing_factor * d->inputtilt_step_size * sign(input_tiltback_target_diff)) +
                ((1 - smoothing_factor) * d->inputtilt_ramped_step_size);
            d->inputtilt_interpolated += d->inputtilt_ramped_step_size;
        }
    } else {
        // Constant step size; no smoothing
        if (fabsf(input_tiltback_target_diff) < d->inputtilt_step_size) {
            d->inputtilt_interpolated = input_tiltback_target;
        } else {
            d->inputtilt_interpolated += d->inputtilt_step_size * sign(input_tiltback_target_diff);
        }
    }

    d->setpoint += d->inputtilt_interpolated;
}

static void apply_turntilt(data *d) {
    if (d->float_conf.turntilt_strength == 0) {
        return;
    }

    float abs_yaw_aggregate = fabsf(d->yaw_aggregate);

    // incremental turn increment since the last iteration
    float turn_increment = d->abs_yaw_change;

    // Minimum threshold based on
    // a) minimum degrees per second (yaw/turn increment)
    // b) minimum yaw aggregate (to filter out wiggling on uneven road)
    if (abs_yaw_aggregate < d->float_conf.turntilt_start_angle || turn_increment < 0.04) {
        d->turntilt_target = 0;
    } else {
        // Calculate desired angle
        float turn_change = d->abs_yaw_change;
        d->turntilt_target = turn_change * d->float_conf.turntilt_strength;

        // Apply speed scaling
        float boost;
        if (d->motor.abs_erpm < d->float_conf.turntilt_erpm_boost_end) {
            boost = 1.0 + d->motor.abs_erpm * d->turntilt_boost_per_erpm;
        } else {
            boost = 1.0 + (float) d->float_conf.turntilt_erpm_boost / 100.0;
        }
        d->turntilt_target *= boost;

        // Increase turntilt based on aggregate yaw change (at most: double it)
        float aggregate_damper = 1.0;
        if (d->motor.abs_erpm < 2000) {
            aggregate_damper = 0.5;
        }
        boost = 1 + aggregate_damper * abs_yaw_aggregate / d->yaw_aggregate_target;
        boost = fminf(boost, 2);
        d->turntilt_target *= boost;

        // Limit angle to max angle
        if (d->turntilt_target > 0) {
            d->turntilt_target = fminf(d->turntilt_target, d->float_conf.turntilt_angle_limit);
        } else {
            d->turntilt_target = fmaxf(d->turntilt_target, -d->float_conf.turntilt_angle_limit);
        }

        // Disable below erpm threshold otherwise add directionality
        if (d->motor.abs_erpm < d->float_conf.turntilt_start_erpm) {
            d->turntilt_target = 0;
        } else {
            d->turntilt_target *= d->motor.erpm_sign;
        }

        // ATR interference: Reduce turntilt_target during moments of high torque response
        float atr_min = 2;
        float atr_max = 5;
        if (sign(d->atr.target_offset) != sign(d->turntilt_target)) {
            // further reduced turntilt during moderate to steep downhills
            atr_min = 1;
            atr_max = 4;
        }
        if (fabsf(d->atr.target_offset) > atr_min) {
            // Start scaling turntilt when ATR>2, down to 0 turntilt for ATR > 5 degrees
            float atr_scaling = (atr_max - fabsf(d->atr.target_offset)) / (atr_max - atr_min);
            if (atr_scaling < 0) {
                atr_scaling = 0;
                // during heavy torque response clear the yaw aggregate too
                d->yaw_aggregate = 0;
            }
            d->turntilt_target *= atr_scaling;
        }
        if (fabsf(d->balance_pitch - d->noseangling_interpolated) > 4) {
            // no setpoint changes during heavy acceleration or braking
            d->turntilt_target = 0;
            d->yaw_aggregate = 0;
        }
    }

    // Move towards target limited by max speed
    rate_limitf(&d->turntilt_interpolated, d->turntilt_target, d->turntilt_step_size);
    d->setpoint += d->turntilt_interpolated;
}

static void brake(data *d) {
    // Brake timeout logic
    float brake_timeout_length = 1;  // Brake Timeout hard-coded to 1s
    if (d->motor.abs_erpm > ERPM_MOVING_THRESHOLD || d->brake_timeout == 0) {
        d->brake_timeout = d->current_time + brake_timeout_length;
    }

    if (d->brake_timeout != 0 && d->current_time > d->brake_timeout) {
        return;
    }

    VESC_IF->timeout_reset();
    VESC_IF->mc_set_brake_current(d->float_conf.brake_current);
}

static void set_current(data *d, float current) {
    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
    VESC_IF->mc_set_current(current);
}

static void imu_ref_callback(float *acc, float *gyro, float *mag, float dt) {
    unused(mag);

    data *d = (data *) ARG;
    balance_filter_update(&d->balance_filter, gyro, acc, dt);
}

static void refloat_thd(void *arg) {
    data *d = (data *) arg;

    configure(d);

    while (!VESC_IF->should_terminate()) {
        beeper_update(d);

        charging_timeout(&d->charging, &d->state);

        d->current_time = VESC_IF->system_time();

        d->pitch = rad2deg(VESC_IF->imu_get_pitch());
        d->roll = rad2deg(VESC_IF->imu_get_roll());
        d->balance_pitch = rad2deg(balance_filter_get_pitch(&d->balance_filter));

        // Darkride:
        if (d->float_conf.fault_darkride_enabled) {
            float abs_roll = fabsf(d->roll);
            if (d->state.darkride) {
                if (abs_roll < 120) {
                    d->state.darkride = false;
                }
            } else if (d->enable_upside_down) {
                if (abs_roll > 150) {
                    d->state.darkride = true;
                    d->is_upside_down_started = false;
                }
            }
        }

        if (d->state.mode == MODE_FLYWHEEL) {
            // flip sign and use offsets
            d->pitch = d->flywheel_pitch_offset - d->pitch;
            d->balance_pitch = d->pitch;
            d->roll -= d->flywheel_roll_offset;
            if (d->roll < -200) {
                d->roll += 360;
            } else if (d->roll > 200) {
                d->roll -= 360;
            }
        } else if (d->state.darkride) {
            d->balance_pitch = -d->balance_pitch - d->darkride_setpoint_correction;
            d->pitch = -d->pitch - d->darkride_setpoint_correction;
        }

        VESC_IF->imu_get_gyro(d->gyro);

        motor_data_update(&d->motor);

        bool remote_connected = false;
        float servo_val = 0;

        switch (d->float_conf.inputtilt_remote_type) {
        case (INPUTTILT_PPM):
            servo_val = VESC_IF->get_ppm();
            remote_connected = VESC_IF->get_ppm_age() < 1;
            break;
        case (INPUTTILT_UART): {
            remote_state remote = VESC_IF->get_remote_state();
            servo_val = remote.js_y;
            remote_connected = remote.age_s < 1;
            break;
        }
        case (INPUTTILT_NONE):
            break;
        }

        if (!remote_connected) {
            servo_val = 0;
        } else {
            // Apply Deadband
            float deadband = d->float_conf.inputtilt_deadband;
            if (fabsf(servo_val) < deadband) {
                servo_val = 0.0;
            } else {
                servo_val = sign(servo_val) * (fabsf(servo_val) - deadband) / (1 - deadband);
            }

            // Invert Throttle
            servo_val *= (d->float_conf.inputtilt_invert_throttle ? -1.0 : 1.0);
        }

        d->throttle_val = servo_val;

        // Turn Tilt:
        d->yaw_angle = VESC_IF->imu_get_yaw() * 180.0f / M_PI;
        float new_change = d->yaw_angle - d->last_yaw_angle;
        bool unchanged = false;
        if ((new_change == 0)  // Exact 0's only happen when the IMU is not updating between loops
            || (fabsf(new_change) > 100))  // yaw flips signs at 180, ignore those changes
        {
            new_change = d->last_yaw_change;
            unchanged = true;
        }
        d->last_yaw_change = new_change;
        d->last_yaw_angle = d->yaw_angle;

        // To avoid overreactions at low speed, limit change here:
        new_change = fminf(new_change, 0.10);
        new_change = fmaxf(new_change, -0.10);
        d->yaw_change = d->yaw_change * 0.8 + 0.2 * (new_change);
        // Clear the aggregate yaw whenever we change direction
        if (sign(d->yaw_change) != sign(d->yaw_aggregate)) {
            d->yaw_aggregate = 0;
        }
        d->abs_yaw_change = fabsf(d->yaw_change);
        // don't count tiny yaw changes towards aggregate
        if ((d->abs_yaw_change > 0.04) && !unchanged) {
            d->yaw_aggregate += d->yaw_change;
        }

        footpad_sensor_update(&d->footpad_sensor, &d->float_conf);

        if (d->footpad_sensor.state == FS_NONE && d->state.state == STATE_RUNNING &&
            d->state.mode != MODE_FLYWHEEL && d->motor.abs_erpm > d->switch_warn_beep_erpm) {
            // If we're at riding speed and the switch is off => ALERT the user
            // set force=true since this could indicate an imminent shutdown/nosedive
            beep_on(d, true);
            d->beep_reason = BEEP_SENSORS;
        } else {
            // if the switch comes back on we stop beeping
            beep_off(d, false);
        }

        float new_pid_value = 0;

        // Control Loop State Logic
        switch (d->state.state) {
        case (STATE_STARTUP):
            // Disable output
            brake(d);
            if (VESC_IF->imu_startup_done()) {
                reset_vars(d);
                // set state to READY so we need to meet start conditions to start
                d->state.state = STATE_READY;

                // if within 5V of LV tiltback threshold, issue 1 beep for each volt below that
                float bat_volts = VESC_IF->mc_get_input_voltage_filtered();
                float threshold = d->float_conf.tiltback_lv + 5;
                if (bat_volts < threshold) {
                    int beeps = (int) fminf(6, threshold - bat_volts);
                    beep_alert(d, beeps + 1, true);
                    d->beep_reason = BEEP_LOWBATT;
                } else {
                    // Let the rider know that the board is ready (one long beep)
                    beep_alert(d, 1, true);
                }
            }
            break;

        case (STATE_RUNNING):
            // Check for faults
            if (check_faults(d)) {
                if (d->state.stop_condition == STOP_SWITCH_FULL && !d->state.darkride) {
                    // dirty landings: add extra margin when rightside up
                    d->startup_pitch_tolerance =
                        d->float_conf.startup_pitch_tolerance + d->startup_pitch_trickmargin;
                    d->fault_angle_pitch_timer = d->current_time;
                }
                break;
            }
            d->odometer_dirty = 1;

            d->enable_upside_down = true;
            d->disengage_timer = d->current_time;

            // Calculate setpoint and interpolation
            calculate_setpoint_target(d);
            calculate_setpoint_interpolated(d);
            d->setpoint = d->setpoint_target_interpolated;
            add_surge(d);
            apply_inputtilt(d);  // Allow Input Tilt for Darkride
            if (!d->state.darkride) {
                // in case of wheelslip, don't change torque tilts, instead slightly decrease each
                // cycle
                if (d->state.wheelslip) {
                    torque_tilt_winddown(&d->torque_tilt);
                    atr_and_braketilt_winddown(&d->atr);
                } else {
                    apply_noseangling(d);
                    apply_turntilt(d);
                    torque_tilt_update(&d->torque_tilt, &d->motor, &d->float_conf);
                    atr_and_braketilt_update(&d->atr, &d->motor, &d->float_conf, d->proportional);
                }

                // aggregated torque tilts:
                // if signs match between torque tilt and ATR + brake tilt, use the more significant
                // one if signs do not match, they are simply added together
                float ab_offset = d->atr.offset + d->atr.braketilt_offset;
                if (sign(ab_offset) == sign(d->torque_tilt.offset)) {
                    d->setpoint +=
                        sign(ab_offset) * fmaxf(fabsf(ab_offset), fabsf(d->torque_tilt.offset));
                } else {
                    d->setpoint += ab_offset + d->torque_tilt.offset;
                }
            }

            // Prepare Brake Scaling (ramp scale values as needed for smooth transitions)
            if (d->motor.abs_erpm < 500) {
                // All scaling should roll back to 1.0x when near a stop for a smooth stand-still
                // and back-forth transition
                d->kp_brake_scale = 0.01 + 0.99 * d->kp_brake_scale;
                d->kp2_brake_scale = 0.01 + 0.99 * d->kp2_brake_scale;
                d->kp_accel_scale = 0.01 + 0.99 * d->kp_accel_scale;
                d->kp2_accel_scale = 0.01 + 0.99 * d->kp2_accel_scale;

            } else if (d->motor.erpm > 0) {
                // Once rolling forward, brakes should transition to scaled values
                d->kp_brake_scale = 0.01 * d->float_conf.kp_brake + 0.99 * d->kp_brake_scale;
                d->kp2_brake_scale = 0.01 * d->float_conf.kp2_brake + 0.99 * d->kp2_brake_scale;
                d->kp_accel_scale = 0.01 + 0.99 * d->kp_accel_scale;
                d->kp2_accel_scale = 0.01 + 0.99 * d->kp2_accel_scale;

            } else {
                // Once rolling backward, the NEW brakes (we will use kp_accel) should transition to
                // scaled values
                d->kp_brake_scale = 0.01 + 0.99 * d->kp_brake_scale;
                d->kp2_brake_scale = 0.01 + 0.99 * d->kp2_brake_scale;
                d->kp_accel_scale = 0.01 * d->float_conf.kp_brake + 0.99 * d->kp_accel_scale;
                d->kp2_accel_scale = 0.01 * d->float_conf.kp2_brake + 0.99 * d->kp2_accel_scale;
            }

            // Do PID maths
            d->proportional = d->setpoint - d->balance_pitch;
            bool tail_down = sign(d->proportional) != d->motor.erpm_sign;

            // Resume real PID maths
            d->integral = d->integral + d->proportional * d->float_conf.ki;

            // Apply I term Filter
            if (d->float_conf.ki_limit > 0 && fabsf(d->integral) > d->float_conf.ki_limit) {
                d->integral = d->float_conf.ki_limit * sign(d->integral);
            }
            // Quickly ramp down integral component during reverse stop
            if (d->state.sat == SAT_REVERSESTOP) {
                d->integral = d->integral * 0.9;
            }

            // Apply P Brake Scaling
            float scaled_kp;
            // Choose appropriate scale based on board angle (this accomodates backwards riding)
            if (d->proportional < 0) {
                scaled_kp = d->float_conf.kp * d->kp_brake_scale;
            } else {
                scaled_kp = d->float_conf.kp * d->kp_accel_scale;
            }

            new_pid_value = scaled_kp * d->proportional + d->integral;

            // Start Rate PID and Booster portion a few cycles later, after the start clicks have
            // been emitted this keeps the start smooth and predictable
            if (d->start_counter_clicks == 0) {

                // Rate P (Angle + Rate, rather than Angle-Rate Cascading)
                float rate_prop = -d->gyro[1];

                float scaled_rate_p;
                // Choose appropriate scale based on board angle (this accomodates backwards riding)
                if (rate_prop < 0) {
                    scaled_rate_p = d->float_conf.kp2 * d->kp2_brake_scale;
                } else {
                    scaled_rate_p = d->float_conf.kp2 * d->kp2_accel_scale;
                }

                d->rate_p = scaled_rate_p * rate_prop;

                // Apply Booster (Now based on True Pitch)
                // Braketilt excluded to allow for soft brakes that strengthen when near tail-drag
                float true_proportional = d->setpoint - d->atr.braketilt_offset - d->pitch;
                float abs_proportional = fabsf(true_proportional);

                float booster_current, booster_angle, booster_ramp;
                if (tail_down) {
                    booster_current = d->float_conf.brkbooster_current;
                    booster_angle = d->float_conf.brkbooster_angle;
                    booster_ramp = d->float_conf.brkbooster_ramp;
                } else {
                    booster_current = d->float_conf.booster_current;
                    booster_angle = d->float_conf.booster_angle;
                    booster_ramp = d->float_conf.booster_ramp;
                }

                // Make booster a bit stronger at higher speed (up to 2x stronger when braking)
                const int boost_min_erpm = 3000;
                if (d->motor.abs_erpm > boost_min_erpm) {
                    float speedstiffness = fminf(1, (d->motor.abs_erpm - boost_min_erpm) / 10000);
                    if (tail_down) {
                        // use higher current at speed when braking
                        booster_current += booster_current * speedstiffness;
                    } else {
                        // when accelerating, we reduce the booster start angle as we get faster
                        // strength remains unchanged
                        float angledivider = 1 + speedstiffness;
                        booster_angle /= angledivider;
                    }
                }

                if (abs_proportional > booster_angle) {
                    if (abs_proportional - booster_angle < booster_ramp) {
                        booster_current *= sign(true_proportional) *
                            ((abs_proportional - booster_angle) / booster_ramp);
                    } else {
                        booster_current *= sign(true_proportional);
                    }
                } else {
                    booster_current = 0;
                }

                // No harsh changes in booster current (effective delay <= 100ms)
                d->applied_booster_current =
                    0.01 * booster_current + 0.99 * d->applied_booster_current;
                d->rate_p += d->applied_booster_current;

                if (d->softstart_pid_limit < d->mc_current_max) {
                    d->rate_p = fminf(fabs(d->rate_p), d->softstart_pid_limit) * sign(d->rate_p);
                    d->softstart_pid_limit += d->softstart_ramp_step_size;
                }

                new_pid_value += d->rate_p;
            } else {
                d->rate_p = 0;
            }

            // Current Limiting!
            float current_limit = d->motor.braking ? d->mc_current_min : d->mc_current_max;
            if (fabsf(new_pid_value) > current_limit) {
                new_pid_value = sign(new_pid_value) * current_limit;
            }

            if (d->traction_control) {
                // freewheel while traction loss is detected
                d->pid_value = 0;
            } else {
                d->pid_value = d->pid_value * 0.8 + new_pid_value * 0.2;
            }

            // Output to motor
            if (d->start_counter_clicks) {
                // Generate alternate pulses to produce distinct "click"
                d->start_counter_clicks--;
                if ((d->start_counter_clicks & 0x1) == 0) {
                    set_current(d, d->pid_value - d->float_conf.startup_click_current);
                } else {
                    set_current(d, d->pid_value + d->float_conf.startup_click_current);
                }
            } else {
                set_current(d, d->pid_value);
            }

            break;

        case (STATE_READY):
            if (d->state.mode == MODE_FLYWHEEL) {
                if (d->flywheel_abort || d->footpad_sensor.state == FS_BOTH) {
                    flywheel_stop(d);
                    break;
                }
            }

            if (d->state.mode != MODE_FLYWHEEL && d->pitch > 75 && d->pitch < 105) {
                if (konami_check(&d->flywheel_konami, &d->footpad_sensor, d->current_time)) {
                    unsigned char enabled[6] = {0x82, 0, 0, 0, 0, 1};
                    cmd_flywheel_toggle(d, enabled, 6);
                }
            }

            if (d->current_time - d->disengage_timer > 10) {
                // 10 seconds of grace period between flipping the board over and allowing darkride
                // mode
                if (d->state.darkride) {
                    beep_alert(d, 1, true);
                }
                d->enable_upside_down = false;
                d->state.darkride = false;
            }
            if (d->current_time - d->disengage_timer > 1800) {  // alert user after 30 minutes
                if (d->current_time - d->nag_timer > 60) {  // beep every 60 seconds
                    d->nag_timer = d->current_time;
                    float input_voltage = VESC_IF->mc_get_input_voltage_filtered();
                    if (input_voltage > d->idle_voltage) {
                        // don't beep if the voltage keeps increasing (board is charging)
                        d->idle_voltage = input_voltage;
                    } else {
                        d->beep_reason = BEEP_IDLE;
                        beep_alert(d, 2, 1);
                    }
                }
            } else {
                d->nag_timer = d->current_time;
                d->idle_voltage = 0;
            }

            if ((d->current_time - d->fault_angle_pitch_timer) > 1) {
                // 1 second after disengaging - set startup tolerance back to normal (aka tighter)
                d->startup_pitch_tolerance = d->float_conf.startup_pitch_tolerance;
            }

            check_odometer(d);

            // Check for valid startup position and switch state
            if (fabsf(d->balance_pitch) < d->startup_pitch_tolerance &&
                fabsf(d->roll) < d->float_conf.startup_roll_tolerance && is_engaged(d)) {
                reset_vars(d);
                break;
            }
            // Ignore roll for the first second while it's upside down
            if (d->state.darkride && (fabsf(d->balance_pitch) < d->startup_pitch_tolerance)) {
                if ((d->current_time - d->disengage_timer) > 1) {
                    // after 1 second:
                    if (fabsf(fabsf(d->roll) - 180) < d->float_conf.startup_roll_tolerance) {
                        reset_vars(d);
                        break;
                    }
                } else {
                    // 1 second or less, ignore roll-faults to allow for kick flips etc.
                    if (d->state.stop_condition != STOP_REVERSE_STOP) {
                        // don't instantly re-engage after a reverse fault!
                        reset_vars(d);
                        break;
                    }
                }
            }
            // Push-start aka dirty landing Part II
            if (d->float_conf.startup_pushstart_enabled && d->motor.abs_erpm > 1000 &&
                is_engaged(d)) {
                if ((fabsf(d->balance_pitch) < 45) && (fabsf(d->roll) < 45)) {
                    // 45 to prevent board engaging when upright or laying sideways
                    // 45 degree tolerance is more than plenty for tricks / extreme mounts
                    reset_vars(d);
                    break;
                }
            }

            // Set RC current or maintain brake current (and keep WDT happy!)
            do_rc_move(d);
            break;
        case (STATE_DISABLED):
            // no set_current, no brake_current
            break;
        }

        VESC_IF->sleep_us(d->loop_time_us);
    }
}

static void write_cfg_to_eeprom(data *d) {
    uint32_t ints = sizeof(RefloatConfig) / 4 + 1;
    uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
    if (!buffer) {
        log_error("Failed to write config to EEPROM: Out of memory.");
        return;
    }

    bool write_ok = true;
    memcpy(buffer, &(d->float_conf), sizeof(RefloatConfig));
    for (uint32_t i = 0; i < ints; i++) {
        eeprom_var v;
        v.as_u32 = buffer[i];
        if (!VESC_IF->store_eeprom_var(&v, i + 1)) {
            write_ok = false;
            break;
        }
    }

    VESC_IF->free(buffer);

    if (write_ok) {
        eeprom_var v;
        v.as_u32 = REFLOATCONFIG_SIGNATURE;
        VESC_IF->store_eeprom_var(&v, 0);
    } else {
        log_error("Failed to write config to EEPROM.");
    }

    beep_alert(d, 1, 0);
}

static void led_thd(void *arg) {
    data *d = (data *) arg;

    while (!VESC_IF->should_terminate()) {
        leds_update(&d->leds, &d->state, d->footpad_sensor.state);
        VESC_IF->sleep_us(1e6 / LEDS_REFRESH_RATE);
    }
}

static void read_cfg_from_eeprom(RefloatConfig *config) {
    uint32_t ints = sizeof(RefloatConfig) / 4 + 1;
    uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
    if (!buffer) {
        log_error("Failed to read config from EEPROM: Out of memory.");
        return;
    }

    eeprom_var v;
    bool read_ok = VESC_IF->read_eeprom_var(&v, 0);
    if (read_ok) {
        if (v.as_u32 == REFLOATCONFIG_SIGNATURE) {
            for (uint32_t i = 0; i < ints; i++) {
                if (!VESC_IF->read_eeprom_var(&v, i + 1)) {
                    read_ok = false;
                    break;
                }
                buffer[i] = v.as_u32;
            }
        } else {
            log_error("Failed signature check while reading config from EEPROM, using defaults.");
            confparser_set_defaults_refloatconfig(config);
            return;
        }
    }

    if (read_ok) {
        memcpy(config, buffer, sizeof(RefloatConfig));
    } else {
        confparser_set_defaults_refloatconfig(config);
        log_error("Failed to read config from EEPROM, using defaults.");
    }

    VESC_IF->free(buffer);
}

static void data_init(data *d) {
    memset(d, 0, sizeof(data));

    read_cfg_from_eeprom(&d->float_conf);

    d->odometer = VESC_IF->mc_get_odometer();

    lcm_init(&d->lcm, &d->float_conf.hardware.leds);
    charging_init(&d->charging);
}

static float app_get_debug(int index) {
    data *d = (data *) ARG;

    switch (index) {
    case (1):
        return d->pid_value;
    case (2):
        return d->proportional;
    case (3):
        return d->integral;
    case (4):
        return d->rate_p;
    case (5):
        return d->setpoint;
    case (6):
        return d->atr.offset;
    case (7):
        return d->motor.erpm;
    case (8):
        return d->motor.current;
    case (9):
        return d->motor.atr_filtered_current;
    default:
        return 0;
    }
}

// See also:
// LcmCommands in lcm.h
// ChargingCommands in charging.h
enum {
    COMMAND_GET_INFO = 0,  // get version / package info
    COMMAND_GET_RTDATA = 1,  // get rt data
    COMMAND_RT_TUNE = 2,  // runtime tuning (don't write to eeprom)
    COMMAND_TUNE_DEFAULTS = 3,  // set tune to defaults (no eeprom)
    COMMAND_CFG_SAVE = 4,  // save config to eeprom
    COMMAND_CFG_RESTORE = 5,  // restore config from eeprom
    COMMAND_TUNE_OTHER = 6,  // make runtime changes to startup/etc
    COMMAND_RC_MOVE = 7,  // move motor while board is idle
    COMMAND_BOOSTER = 8,  // change booster settings
    COMMAND_PRINT_INFO = 9,  // print verbose info
    COMMAND_GET_ALLDATA = 10,  // send all data, compact
    COMMAND_EXPERIMENT = 11,  // generic cmd for sending data, used for testing/tuning new features
    COMMAND_LOCK = 12,
    COMMAND_HANDTEST = 13,
    COMMAND_TUNE_TILT = 14,
    COMMAND_FLYWHEEL = 22,

    // commands above 200 are unstable and can change protocol at any time
    COMMAND_GET_RTDATA_2 = 201,
    COMMAND_LIGHTS_CONTROL = 202,
} Commands;

static void send_realtime_data(data *d) {
    static const int bufsize = 72;
    uint8_t buffer[bufsize];
    int32_t ind = 0;
    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_GET_RTDATA;

    // RT Data
    buffer_append_float32_auto(buffer, d->pid_value, &ind);
    buffer_append_float32_auto(buffer, d->balance_pitch, &ind);
    buffer_append_float32_auto(buffer, d->roll, &ind);

    uint8_t state = (state_compat(&d->state) & 0xF);
    buffer[ind++] = (state & 0xF) + (sat_compat(&d->state) << 4);
    state = footpad_sensor_state_to_switch_compat(d->footpad_sensor.state);
    if (d->state.mode == MODE_HANDTEST) {
        state |= 0x8;
    }
    buffer[ind++] = (state & 0xF) + (d->beep_reason << 4);
    buffer_append_float32_auto(buffer, d->footpad_sensor.adc1, &ind);
    buffer_append_float32_auto(buffer, d->footpad_sensor.adc2, &ind);

    // Setpoints
    buffer_append_float32_auto(buffer, d->setpoint, &ind);
    buffer_append_float32_auto(buffer, d->atr.offset, &ind);
    buffer_append_float32_auto(buffer, d->atr.braketilt_offset, &ind);
    buffer_append_float32_auto(buffer, d->torque_tilt.offset, &ind);
    buffer_append_float32_auto(buffer, d->turntilt_interpolated, &ind);
    buffer_append_float32_auto(buffer, d->inputtilt_interpolated, &ind);

    // DEBUG
    buffer_append_float32_auto(buffer, d->pitch, &ind);
    buffer_append_float32_auto(buffer, d->motor.atr_filtered_current, &ind);
    buffer_append_float32_auto(buffer, d->atr.accel_diff, &ind);
    if (d->state.charging) {
        buffer_append_float32_auto(buffer, d->charging.current, &ind);
        buffer_append_float32_auto(buffer, d->charging.voltage, &ind);
    } else {
        buffer_append_float32_auto(buffer, d->applied_booster_current, &ind);
        buffer_append_float32_auto(buffer, d->motor.current, &ind);
    }
    buffer_append_float32_auto(buffer, d->throttle_val, &ind);

    SEND_APP_DATA(buffer, bufsize, ind);
}

static void cmd_send_all_data(data *d, unsigned char mode) {
    static const int bufsize = 60;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_GET_ALLDATA;

    mc_fault_code fault = VESC_IF->mc_get_fault();
    if (fault != FAULT_CODE_NONE) {
        buffer[ind++] = 69;
        buffer[ind++] = fault;
    } else {
        buffer[ind++] = mode;

        // RT Data
        buffer_append_float16(buffer, d->pid_value, 10, &ind);
        buffer_append_float16(buffer, d->balance_pitch, 10, &ind);
        buffer_append_float16(buffer, d->roll, 10, &ind);

        uint8_t state = (state_compat(&d->state) & 0xF) + (sat_compat(&d->state) << 4);
        buffer[ind++] = state;

        // passed switch-state includes bit3 for handtest, and bits4..7 for beep reason
        state = footpad_sensor_state_to_switch_compat(d->footpad_sensor.state);
        if (d->state.mode == MODE_HANDTEST) {
            state |= 0x8;
        }
        buffer[ind++] = (state & 0xF) + (d->beep_reason << 4);
        d->beep_reason = BEEP_NONE;

        buffer[ind++] = d->footpad_sensor.adc1 * 50;
        buffer[ind++] = d->footpad_sensor.adc2 * 50;

        // Setpoints (can be positive or negative)
        buffer[ind++] = d->setpoint * 5 + 128;
        buffer[ind++] = d->atr.offset * 5 + 128;
        buffer[ind++] = d->atr.braketilt_offset * 5 + 128;
        buffer[ind++] = d->torque_tilt.offset * 5 + 128;
        buffer[ind++] = d->turntilt_interpolated * 5 + 128;
        buffer[ind++] = d->inputtilt_interpolated * 5 + 128;

        buffer_append_float16(buffer, d->pitch, 10, &ind);
        buffer[ind++] = d->applied_booster_current + 128;

        // Now send motor stuff:
        buffer_append_float16(buffer, VESC_IF->mc_get_input_voltage_filtered(), 10, &ind);
        buffer_append_int16(buffer, VESC_IF->mc_get_rpm(), &ind);
        buffer_append_float16(buffer, VESC_IF->mc_get_speed(), 10, &ind);
        buffer_append_float16(buffer, VESC_IF->mc_get_tot_current(), 10, &ind);
        buffer_append_float16(buffer, VESC_IF->mc_get_tot_current_in(), 10, &ind);
        buffer[ind++] = VESC_IF->mc_get_duty_cycle_now() * 100 + 128;
        if (VESC_IF->foc_get_id != NULL) {
            buffer[ind++] = fabsf(VESC_IF->foc_get_id()) * 3;
        } else {
            // using 222 as magic number to avoid false positives with 255
            buffer[ind++] = 222;
            // ind = 35
        }

        if (mode >= 2) {
            // data not required as fast as possible
            buffer_append_float32_auto(buffer, VESC_IF->mc_get_distance_abs(), &ind);
            buffer[ind++] = fmaxf(0, VESC_IF->mc_temp_fet_filtered() * 2);
            buffer[ind++] = fmaxf(0, VESC_IF->mc_temp_motor_filtered() * 2);
            buffer[ind++] = 0;  // fmaxf(VESC_IF->mc_batt_temp() * 2);
            // ind = 42
        }
        if (mode >= 3) {
            // data required even less frequently
            buffer_append_uint32(buffer, VESC_IF->mc_get_odometer(), &ind);
            buffer_append_float16(buffer, VESC_IF->mc_get_amp_hours(false), 10, &ind);
            buffer_append_float16(buffer, VESC_IF->mc_get_amp_hours_charged(false), 10, &ind);
            buffer_append_float16(buffer, VESC_IF->mc_get_watt_hours(false), 1, &ind);
            buffer_append_float16(buffer, VESC_IF->mc_get_watt_hours_charged(false), 1, &ind);
            buffer[ind++] = fmaxf(0, fminf(125, VESC_IF->mc_get_battery_level(NULL))) * 2;
            // ind = 55
        }
        if (mode >= 4) {
            // make charge current and voltage available in mode 4
            buffer_append_float16(buffer, d->charging.current, 10, &ind);
            buffer_append_float16(buffer, d->charging.voltage, 10, &ind);
            // ind = 59
        }
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

static void split(unsigned char byte, int *h1, int *h2) {
    *h1 = byte & 0xF;
    *h2 = byte >> 4;
}

static void cmd_print_info(data *d) {
    unused(d);
}

static void cmd_lock(data *d, unsigned char *cfg) {
    if (d->state.state < STATE_RUNNING) {
        // restore config before locking to avoid accidentally writing temporary changes
        read_cfg_from_eeprom(&d->float_conf);
        d->float_conf.disabled = cfg[0] ? true : false;
        d->state.state = cfg[0] ? STATE_DISABLED : STATE_STARTUP;
        write_cfg_to_eeprom(d);
    }
}

static void cmd_handtest(data *d, unsigned char *cfg) {
    if (d->state.state != STATE_READY) {
        return;
    }

    if (d->state.mode != MODE_NORMAL && d->state.mode != MODE_HANDTEST) {
        return;
    }

    d->state.mode = cfg[0] ? MODE_HANDTEST : MODE_NORMAL;
    if (d->state.mode == MODE_HANDTEST) {
        // temporarily reduce max currents to make hand test safer / gentler
        d->mc_current_max = d->mc_current_min = 7;
        // Disable I-term and all tune modifiers and tilts
        d->float_conf.ki = 0;
        d->float_conf.kp_brake = 1;
        d->float_conf.kp2_brake = 1;
        d->float_conf.brkbooster_angle = 100;
        d->float_conf.booster_angle = 100;
        d->float_conf.torquetilt_strength = 0;
        d->float_conf.torquetilt_strength_regen = 0;
        d->float_conf.atr_strength_up = 0;
        d->float_conf.atr_strength_down = 0;
        d->float_conf.turntilt_strength = 0;
        d->float_conf.tiltback_constant = 0;
        d->float_conf.tiltback_variable = 0;
        d->float_conf.fault_delay_pitch = 50;
        d->float_conf.fault_delay_roll = 50;
    } else {
        read_cfg_from_eeprom(&d->float_conf);
        configure(d);
    }
}

static void cmd_experiment(data *d, unsigned char *cfg) {
    d->surge_angle = cfg[0];
    d->surge_angle /= 10;
    d->float_conf.surge_duty_start = cfg[1];
    d->float_conf.surge_duty_start /= 100;
    if ((d->surge_angle > 1) || (d->float_conf.surge_duty_start < 0.85)) {
        d->float_conf.surge_duty_start = 0.85;
        d->surge_angle = 0.6;
    } else {
        d->surge_enable = true;
        beep_alert(d, 2, 0);
    }
    d->surge_angle2 = d->surge_angle * 2;
    d->surge_angle3 = d->surge_angle * 3;

    if (d->surge_angle == 0) {
        d->surge_enable = false;
    }
}

static void cmd_booster(data *d, unsigned char *cfg) {
    int h1, h2;
    split(cfg[0], &h1, &h2);
    d->float_conf.booster_angle = h1 + 5;
    d->float_conf.booster_ramp = h2 + 2;

    split(cfg[1], &h1, &h2);
    if (h1 == 0) {
        d->float_conf.booster_current = 0;
    } else {
        d->float_conf.booster_current = 8 + h1 * 2;
    }

    split(cfg[2], &h1, &h2);
    d->float_conf.brkbooster_angle = h1 + 5;
    d->float_conf.brkbooster_ramp = h2 + 2;

    split(cfg[3], &h1, &h2);
    if (h1 == 0) {
        d->float_conf.brkbooster_current = 0;
    } else {
        d->float_conf.brkbooster_current = 8 + h1 * 2;
    }

    beep_alert(d, 1, false);
}

/**
 * cmd_runtime_tune		Extract tune info from 20byte message but don't write to EEPROM!
 */
static void cmd_runtime_tune(data *d, unsigned char *cfg, int len) {
    int h1, h2;
    if (len >= 12) {
        split(cfg[0], &h1, &h2);
        d->float_conf.kp = h1 + 15;
        d->float_conf.kp2 = ((float) h2) / 10;

        split(cfg[1], &h1, &h2);
        d->float_conf.ki = h1;
        if (h1 == 1) {
            d->float_conf.ki = 0.005;
        } else if (h1 > 1) {
            d->float_conf.ki = ((float) (h1 - 1)) / 100;
        }
        d->float_conf.ki_limit = h2 + 19;
        if (h2 == 0) {
            d->float_conf.ki_limit = 0;
        }

        split(cfg[2], &h1, &h2);
        d->float_conf.booster_angle = h1 + 5;
        d->float_conf.booster_ramp = h2 + 2;

        split(cfg[3], &h1, &h2);
        if (h1 == 0) {
            d->float_conf.booster_current = 0;
        } else {
            d->float_conf.booster_current = 8 + h1 * 2;
        }
        d->float_conf.turntilt_strength = h2;

        split(cfg[4], &h1, &h2);
        d->float_conf.turntilt_angle_limit = (h1 & 0x3) + 2;
        d->float_conf.turntilt_start_erpm = (float) (h1 >> 2) * 500 + 1000;
        d->float_conf.mahony_kp = ((float) h2) / 10 + 1.5;

        split(cfg[5], &h1, &h2);
        if (h1 == 0) {
            d->float_conf.atr_strength_up = 0;
        } else {
            d->float_conf.atr_strength_up = ((float) h1) / 10.0 + 0.5;
        }
        if (h2 == 0) {
            d->float_conf.atr_strength_down = 0;
        } else {
            d->float_conf.atr_strength_down = ((float) h2) / 10.0 + 0.5;
        }

        split(cfg[6], &h1, &h2);
        d->float_conf.atr_speed_boost = ((float) (h2 * 5)) / 100;
        if (h1 != 0) {
            d->float_conf.atr_speed_boost *= -1;
        }

        split(cfg[7], &h1, &h2);
        d->float_conf.atr_angle_limit = h1 + 5;
        d->float_conf.atr_on_speed = (h2 & 0x3) + 3;
        d->float_conf.atr_off_speed = (h2 >> 2) + 2;

        split(cfg[8], &h1, &h2);
        d->float_conf.atr_response_boost = ((float) h1) / 10 + 1;
        d->float_conf.atr_transition_boost = ((float) h2) / 5 + 1;

        split(cfg[9], &h1, &h2);
        d->float_conf.atr_amps_accel_ratio = h1 + 5;
        d->float_conf.atr_amps_decel_ratio = h2 + 5;

        split(cfg[10], &h1, &h2);
        d->float_conf.braketilt_strength = h1;
        d->float_conf.braketilt_lingering = h2;

        split(cfg[11], &h1, &h2);
        d->mc_current_max = h1 * 5 + 55;
        d->mc_current_min = h2 * 5 + 55;
        if (h1 == 0) {
            d->mc_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
        }
        if (h2 == 0) {
            d->mc_current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
        }

        d->turntilt_step_size = d->float_conf.turntilt_speed / d->float_conf.hertz;
    }
    if (len >= 16) {
        split(cfg[12], &h1, &h2);
        float thup = h1;
        float thdown = h2;
        d->float_conf.atr_threshold_up = thup / 2;
        d->float_conf.atr_threshold_down = thdown / 2;

        split(cfg[13], &h1, &h2);
        float ttup = h1;
        float ttdn = h2;
        d->float_conf.torquetilt_strength = ttup / 10 * 0.3;
        d->float_conf.torquetilt_strength_regen = ttdn / 10 * 0.3;

        split(cfg[14], &h1, &h2);
        float maxangle = h1;
        d->float_conf.torquetilt_start_current = h2 + 15;
        d->float_conf.torquetilt_angle_limit = maxangle / 2;

        split(cfg[15], &h1, &h2);
        float onspd = h1;
        float offspd = h2;
        d->float_conf.torquetilt_on_speed = onspd / 2;
        d->float_conf.torquetilt_off_speed = offspd + 3;
    }
    if (len >= 17) {
        split(cfg[16], &h1, &h2);
        d->float_conf.kp_brake = ((float) h1 + 1) / 10;
        d->float_conf.kp2_brake = ((float) h2) / 10;
        beep_alert(d, 1, 1);
    }

    reconfigure(d);
}

static void cmd_tune_defaults(data *d) {
    d->float_conf.kp = CFG_DFLT_KP;
    d->float_conf.kp2 = CFG_DFLT_KP2;
    d->float_conf.ki = CFG_DFLT_KI;
    d->float_conf.mahony_kp = CFG_DFLT_MAHONY_KP;
    d->float_conf.mahony_kp_roll = CFG_DFLT_MAHONY_KP_ROLL;
    d->float_conf.bf_accel_confidence_decay = CFG_DFLT_BF_ACCEL_CONFIDENCE_DECAY;
    d->float_conf.kp_brake = CFG_DFLT_KP_BRAKE;
    d->float_conf.kp2_brake = CFG_DFLT_KP2_BRAKE;
    d->float_conf.ki_limit = CFG_DFLT_KI_LIMIT;
    d->float_conf.booster_angle = CFG_DFLT_BOOSTER_ANGLE;
    d->float_conf.booster_ramp = CFG_DFLT_BOOSTER_RAMP;
    d->float_conf.booster_current = CFG_DFLT_BOOSTER_CURRENT;
    d->float_conf.brkbooster_angle = CFG_DFLT_BRKBOOSTER_ANGLE;
    d->float_conf.brkbooster_ramp = CFG_DFLT_BRKBOOSTER_RAMP;
    d->float_conf.brkbooster_current = CFG_DFLT_BRKBOOSTER_CURRENT;
    d->float_conf.turntilt_strength = CFG_DFLT_TURNTILT_STRENGTH;
    d->float_conf.turntilt_angle_limit = CFG_DFLT_TURNTILT_ANGLE_LIMIT;
    d->float_conf.turntilt_start_angle = CFG_DFLT_TURNTILT_START_ANGLE;
    d->float_conf.turntilt_start_erpm = CFG_DFLT_TURNTILT_START_ERPM;
    d->float_conf.turntilt_speed = CFG_DFLT_TURNTILT_SPEED;
    d->float_conf.turntilt_erpm_boost = CFG_DFLT_TURNTILT_ERPM_BOOST;
    d->float_conf.turntilt_erpm_boost_end = CFG_DFLT_TURNTILT_ERPM_BOOST_END;
    d->float_conf.turntilt_yaw_aggregate = CFG_DFLT_TURNTILT_YAW_AGGREGATE;
    d->float_conf.atr_strength_up = CFG_DFLT_ATR_UPHILL_STRENGTH;
    d->float_conf.atr_strength_down = CFG_DFLT_ATR_DOWNHILL_STRENGTH;
    d->float_conf.atr_threshold_up = CFG_DFLT_ATR_THRESHOLD_UP;
    d->float_conf.atr_threshold_down = CFG_DFLT_ATR_THRESHOLD_DOWN;
    d->float_conf.atr_speed_boost = CFG_DFLT_ATR_SPEED_BOOST;
    d->float_conf.atr_angle_limit = CFG_DFLT_ATR_ANGLE_LIMIT;
    d->float_conf.atr_on_speed = CFG_DFLT_ATR_ON_SPEED;
    d->float_conf.atr_off_speed = CFG_DFLT_ATR_OFF_SPEED;
    d->float_conf.atr_response_boost = CFG_DFLT_ATR_RESPONSE_BOOST;
    d->float_conf.atr_transition_boost = CFG_DFLT_ATR_TRANSITION_BOOST;
    d->float_conf.atr_filter = CFG_DFLT_ATR_FILTER;
    d->float_conf.atr_amps_accel_ratio = CFG_DFLT_ATR_AMPS_ACCEL_RATIO;
    d->float_conf.atr_amps_decel_ratio = CFG_DFLT_ATR_AMPS_DECEL_RATIO;
    d->float_conf.braketilt_strength = CFG_DFLT_BRAKETILT_STRENGTH;
    d->float_conf.braketilt_lingering = CFG_DFLT_BRAKETILT_LINGERING;

    d->float_conf.startup_pitch_tolerance = CFG_DFLT_STARTUP_PITCH_TOLERANCE;
    d->float_conf.startup_roll_tolerance = CFG_DFLT_STARTUP_ROLL_TOLERANCE;
    d->float_conf.startup_speed = CFG_DFLT_STARTUP_SPEED;
    d->float_conf.startup_click_current = CFG_DFLT_STARTUP_CLICK_CURRENT;
    d->float_conf.brake_current = CFG_DFLT_BRAKE_CURRENT;
    d->float_conf.is_beeper_enabled = CFG_DFLT_IS_BEEPER_ENABLED;
    d->float_conf.tiltback_constant = CFG_DFLT_TILTBACK_CONSTANT;
    d->float_conf.tiltback_constant_erpm = CFG_DFLT_TILTBACK_CONSTANT_ERPM;
    d->float_conf.tiltback_variable = CFG_DFLT_TILTBACK_VARIABLE;
    d->float_conf.tiltback_variable_max = CFG_DFLT_TILTBACK_VARIABLE_MAX;
    d->float_conf.noseangling_speed = CFG_DFLT_NOSEANGLING_SPEED;
    d->float_conf.startup_pushstart_enabled = CFG_DFLT_PUSHSTART_ENABLED;
    d->float_conf.startup_simplestart_enabled = CFG_DFLT_SIMPLESTART_ENABLED;
    d->float_conf.startup_dirtylandings_enabled = CFG_DFLT_DIRTYLANDINGS_ENABLED;

    // Update values normally done in configure()
    d->turntilt_step_size = d->float_conf.turntilt_speed / d->float_conf.hertz;

    d->startup_step_size = d->float_conf.startup_speed / d->float_conf.hertz;
    d->noseangling_step_size = d->float_conf.noseangling_speed / d->float_conf.hertz;
    d->startup_pitch_trickmargin = d->float_conf.startup_dirtylandings_enabled ? 10 : 0;
    d->tiltback_variable = d->float_conf.tiltback_variable / 1000;
    if (d->tiltback_variable > 0) {
        d->tiltback_variable_max_erpm =
            fabsf(d->float_conf.tiltback_variable_max / d->tiltback_variable);
    } else {
        d->tiltback_variable_max_erpm = 100000;
    }

    reconfigure(d);
}

/**
 * cmd_runtime_tune_tilt: Extract settings from 20byte message but don't write to EEPROM!
 */
static void cmd_runtime_tune_tilt(data *d, unsigned char *cfg, int len) {
    unsigned int flags = cfg[0];
    bool duty_beep = flags & 0x1;
    d->float_conf.is_dutybeep_enabled = duty_beep;
    float retspeed = cfg[1];
    if (retspeed > 0) {
        d->float_conf.tiltback_return_speed = retspeed / 10;
        d->tiltback_return_step_size = d->float_conf.tiltback_return_speed / d->float_conf.hertz;
    }
    d->float_conf.tiltback_duty = (float) cfg[2] / 100.0;
    d->float_conf.tiltback_duty_angle = (float) cfg[3] / 10.0;
    d->float_conf.tiltback_duty_speed = (float) cfg[4] / 10.0;

    if (len >= 6) {
        float surge_duty_start = cfg[5];
        if (surge_duty_start > 0) {
            d->float_conf.surge_duty_start = surge_duty_start / 100.0;
            d->float_conf.surge_angle = (float) cfg[6] / 20.0;
            d->surge_angle = d->float_conf.surge_angle;
            d->surge_angle2 = d->float_conf.surge_angle * 2;
            d->surge_angle3 = d->float_conf.surge_angle * 3;
            d->surge_enable = d->surge_angle > 0;
        }
        beep_alert(d, 1, 1);
    } else {
        beep_alert(d, 3, 0);
    }
}

/**
 * cmd_runtime_tune_other		Extract settings from 20byte message but don't write to
 * EEPROM!
 */
static void cmd_runtime_tune_other(data *d, unsigned char *cfg, int len) {
    unsigned int flags = cfg[0];
    d->beeper_enabled = ((flags & 0x2) == 2);
    d->float_conf.fault_reversestop_enabled = ((flags & 0x4) == 4);
    d->float_conf.fault_is_dual_switch = ((flags & 0x8) == 8);
    d->float_conf.fault_darkride_enabled = ((flags & 0x10) == 0x10);
    bool dirty_landings = ((flags & 0x20) == 0x20);
    d->float_conf.startup_simplestart_enabled = ((flags & 0x40) == 0x40);
    d->float_conf.startup_pushstart_enabled = ((flags & 0x80) == 0x80);

    d->float_conf.is_beeper_enabled = d->beeper_enabled;
    d->startup_pitch_trickmargin = dirty_landings ? 10 : 0;
    d->float_conf.startup_dirtylandings_enabled = dirty_landings;

    // startup
    float ctrspeed = cfg[1];
    float pitchtolerance = cfg[2];
    float rolltolerance = cfg[3];
    float brakecurrent = cfg[4];
    float clickcurrent = cfg[5];

    d->startup_step_size = ctrspeed / d->float_conf.hertz;
    d->float_conf.startup_speed = ctrspeed;
    d->float_conf.startup_pitch_tolerance = pitchtolerance / 10;
    d->float_conf.startup_roll_tolerance = rolltolerance;
    d->float_conf.brake_current = brakecurrent / 2;
    d->float_conf.startup_click_current = clickcurrent;

    // nose angling
    float tiltconst = cfg[6] - 100;
    float tiltspeed = cfg[7];
    int tilterpm = cfg[8] * 100;
    float tiltvarrate = cfg[9];
    float tiltvarmax = cfg[10];

    if (fabsf(tiltconst) <= 20) {
        d->float_conf.tiltback_constant = tiltconst / 2;
        d->float_conf.tiltback_constant_erpm = tilterpm;
        if (tiltspeed > 0) {
            d->noseangling_step_size = tiltspeed / 10 / d->float_conf.hertz;
            d->float_conf.noseangling_speed = tiltspeed / 10;
        }
        d->float_conf.tiltback_variable = tiltvarrate / 100;
        d->float_conf.tiltback_variable_max = tiltvarmax / 10;

        d->startup_step_size = d->float_conf.startup_speed / d->float_conf.hertz;
        d->noseangling_step_size = d->float_conf.noseangling_speed / d->float_conf.hertz;
        d->tiltback_variable = d->float_conf.tiltback_variable / 1000;
        if (d->tiltback_variable > 0) {
            d->tiltback_variable_max_erpm =
                fabsf(d->float_conf.tiltback_variable_max / d->tiltback_variable);
        } else {
            d->tiltback_variable_max_erpm = 100000;
        }
        d->float_conf.tiltback_variable_erpm = cfg[11] * 100;
    }

    if (len >= 14) {
        int inputtilt = cfg[12] & 0x3;
        if (inputtilt <= INPUTTILT_PPM) {
            d->float_conf.inputtilt_remote_type = inputtilt;
            if (inputtilt > 0) {
                d->float_conf.inputtilt_angle_limit = cfg[12] >> 2;
                d->float_conf.inputtilt_speed = cfg[13];
                d->inputtilt_step_size = d->float_conf.inputtilt_speed / d->float_conf.hertz;
            }
        }
    }
}

void cmd_rc_move(data *d, unsigned char *cfg) {
    int ind = 0;
    int direction = cfg[ind++];
    int current = cfg[ind++];
    int time = cfg[ind++];
    int sum = cfg[ind++];
    if (sum != time + current) {
        current = 0;
    } else if (direction == 0) {
        current = -current;
    }

    if (d->state.state == STATE_READY) {
        d->rc_counter = 0;
        if (current == 0) {
            d->rc_steps = 1;
            d->rc_current_target = 0;
            d->rc_current = 0;
        } else {
            d->rc_steps = time * 100;
            d->rc_current_target = current / 10.0;
            if (d->rc_current_target > 8) {
                d->rc_current_target = 2;
            }
        }
    }
}

static void cmd_flywheel_toggle(data *d, unsigned char *cfg, int len) {
    if ((cfg[0] & 0x80) == 0) {
        return;
    }

    // Only proceed in NORMAL or FLYWHEEL mode
    if (d->state.mode != MODE_NORMAL && d->state.mode != MODE_FLYWHEEL) {
        return;
    }

    // If state is not READY, only proceed if mode is FLYWHEEL
    // (i.e. don't allow to turn FLYWHEEL on in any other mode than READY)
    if (d->state.state != STATE_READY && d->state.mode != MODE_FLYWHEEL) {
        return;
    }

    // cfg[0]: Command (0 = stop, 1 = start)
    // All of the below are mandatory, but can be 0 to just use defaults
    // cfg[1]: AngleP: kp*10 (e.g. 90: P=9.0)
    // cfg[2]: RateP:  kp2*100 (e.g. 50: RateP=0.5)
    // cfg[3]: Duty TB Angle * 10
    // cfg[4]: Duty TB Duty %
    // cfg[5]: Allow Abort via footpad (1 = do allow, 0 = don't allow)
    // Optional:
    // cfg[6]: Duty TB Speed in deg/sec
    int command = cfg[0] & 0x7F;
    d->state.mode = command == 0 ? MODE_NORMAL : MODE_FLYWHEEL;
    if (d->state.mode == MODE_FLYWHEEL) {
        if ((d->flywheel_pitch_offset == 0) || (command == 2)) {
            // accidental button press?? board isn't evn close to being upright
            if (fabsf(d->pitch) < 70) {
                d->state.mode = MODE_NORMAL;
                return;
            }

            d->flywheel_pitch_offset = d->pitch;
            d->flywheel_roll_offset = d->roll;
            beep_alert(d, 1, 1);
        } else {
            beep_alert(d, 3, 0);
        }
        d->flywheel_abort = false;

        // Tighter startup/fault tolerances
        d->startup_pitch_tolerance = 0.2;
        d->float_conf.startup_pitch_tolerance = 0.2;
        d->float_conf.startup_roll_tolerance = 25;
        d->float_conf.fault_pitch = 6;
        d->float_conf.fault_roll = 35;  // roll can fluctuate significantly in the upright position
        if (command & 0x4) {
            d->float_conf.fault_roll = 90;
        }
        d->float_conf.fault_delay_pitch = 50;  // 50ms delay should help filter out IMU noise
        d->float_conf.fault_delay_roll = 50;  // 50ms delay should help filter out IMU noise
        d->surge_enable = false;

        // Aggressive P with some D (aka Rate-P) for Mahony kp=0.3
        d->float_conf.kp = 8.0;
        d->float_conf.kp2 = 0.3;

        if (cfg[1] > 0) {
            d->float_conf.kp = cfg[1];
            d->float_conf.kp /= 10;
        }
        if (cfg[2] > 0) {
            d->float_conf.kp2 = cfg[2];
            d->float_conf.kp2 /= 100;
        }

        d->float_conf.tiltback_duty_angle = 2;
        d->float_conf.tiltback_duty = 0.1;
        d->float_conf.tiltback_duty_speed = 5;
        d->float_conf.tiltback_return_speed = 5;

        if (cfg[3] > 0) {
            d->float_conf.tiltback_duty_angle = cfg[3];
            d->float_conf.tiltback_duty_angle /= 10;
        }
        if (cfg[4] > 0) {
            d->float_conf.tiltback_duty = cfg[4];
            d->float_conf.tiltback_duty /= 100;
        }
        if ((len > 6) && (cfg[6] > 1) && (cfg[6] < 100)) {
            d->float_conf.tiltback_duty_speed = cfg[6] / 2;
            d->float_conf.tiltback_return_speed = cfg[6] / 2;
        }
        d->tiltback_duty_step_size = d->float_conf.tiltback_duty_speed / d->float_conf.hertz;
        d->tiltback_return_step_size = d->float_conf.tiltback_return_speed / d->float_conf.hertz;

        // Limit speed of wheel and limit amps
        VESC_IF->set_cfg_float(CFG_PARAM_l_min_erpm + 100, -6000);
        VESC_IF->set_cfg_float(CFG_PARAM_l_max_erpm + 100, 6000);
        d->mc_current_max = d->mc_current_min = 40;

        // d->flywheel_allow_abort = cfg[5];

        // Disable I-term and all tune modifiers and tilts
        d->float_conf.ki = 0;
        d->float_conf.kp_brake = 1;
        d->float_conf.kp2_brake = 1;
        d->float_conf.brkbooster_angle = 100;
        d->float_conf.booster_angle = 100;
        d->float_conf.torquetilt_strength = 0;
        d->float_conf.torquetilt_strength_regen = 0;
        d->float_conf.atr_strength_up = 0;
        d->float_conf.atr_strength_down = 0;
        d->float_conf.turntilt_strength = 0;
        d->float_conf.tiltback_constant = 0;
        d->float_conf.tiltback_variable = 0;
        d->float_conf.brake_current = 0;
        d->float_conf.fault_darkride_enabled = false;
        d->float_conf.fault_reversestop_enabled = false;
        d->float_conf.tiltback_constant = 0;
        d->tiltback_variable_max_erpm = 0;
        d->tiltback_variable = 0;
    } else {
        flywheel_stop(d);
    }
}

void flywheel_stop(data *d) {
    beep_on(d, 1);
    d->state.mode = MODE_NORMAL;
    read_cfg_from_eeprom(&d->float_conf);
    configure(d);
}

static void send_realtime_data2(data *d) {
    static const int bufsize = 75;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_GET_RTDATA_2;

    // mask indicates what groups of data are sent, to prevent sending data
    // that are not useful in a given state
    uint8_t mask = 0;
    if (d->state.state == STATE_RUNNING) {
        mask |= 0x1;
    }

    if (d->state.charging) {
        mask |= 0x2;
    }

    buffer[ind++] = mask;

    buffer[ind++] = d->state.mode << 4 | d->state.state;

    uint8_t flags = d->state.charging << 5 | d->state.darkride << 1 | d->state.wheelslip;
    buffer[ind++] = d->footpad_sensor.state << 6 | flags;

    buffer[ind++] = d->state.sat << 4 | d->state.stop_condition;

    buffer[ind++] = d->beep_reason;

    buffer_append_float32_auto(buffer, d->pitch, &ind);
    buffer_append_float32_auto(buffer, d->balance_pitch, &ind);
    buffer_append_float32_auto(buffer, d->roll, &ind);

    buffer_append_float32_auto(buffer, d->footpad_sensor.adc1, &ind);
    buffer_append_float32_auto(buffer, d->footpad_sensor.adc2, &ind);
    buffer_append_float32_auto(buffer, d->throttle_val, &ind);

    if (d->state.state == STATE_RUNNING) {
        // Setpoints
        buffer_append_float32_auto(buffer, d->setpoint, &ind);
        buffer_append_float32_auto(buffer, d->atr.offset, &ind);
        buffer_append_float32_auto(buffer, d->atr.braketilt_offset, &ind);
        buffer_append_float32_auto(buffer, d->torque_tilt.offset, &ind);
        buffer_append_float32_auto(buffer, d->turntilt_interpolated, &ind);
        buffer_append_float32_auto(buffer, d->inputtilt_interpolated, &ind);

        // DEBUG
        buffer_append_float32_auto(buffer, d->pid_value, &ind);
        buffer_append_float32_auto(buffer, d->motor.atr_filtered_current, &ind);
        buffer_append_float32_auto(buffer, d->atr.accel_diff, &ind);
        buffer_append_float32_auto(buffer, d->atr.speed_boost, &ind);
        buffer_append_float32_auto(buffer, d->applied_booster_current, &ind);
    }

    if (d->state.charging) {
        buffer_append_float32_auto(buffer, d->charging.current, &ind);
        buffer_append_float32_auto(buffer, d->charging.voltage, &ind);
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

static void lights_control_request(CfgLeds *leds, uint8_t *buffer, size_t len, LcmData *lcm) {
    if (len < 2) {
        return;
    }

    uint8_t mask = buffer[0];
    uint8_t value = buffer[1];

    if (mask != 0) {
        if (mask & 0x1) {
            leds->on = value & 0x1;
        }

        if (mask & 0x2) {
            leds->headlights_on = value & 0x2;
        }

        if (lcm->enabled) {
            lcm_configure(lcm, leds);
        }
    }
}

static void lights_control_response(const CfgLeds *leds) {
    static const int bufsize = 3;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LIGHTS_CONTROL;
    buffer[ind++] = leds->headlights_on << 1 | leds->on;

    SEND_APP_DATA(buffer, bufsize, ind);
}

// Handler for incoming app commands
static void on_command_received(unsigned char *buffer, unsigned int len) {
    data *d = (data *) ARG;
    uint8_t magicnr = buffer[0];
    uint8_t command = buffer[1];

    if (len < 2) {
        log_error("Received command data too short.");
        return;
    }
    if (magicnr != 101) {
        log_error("Invalid Package ID: %u", magicnr);
        return;
    }

    switch (command) {
    case COMMAND_GET_INFO: {
        int32_t ind = 0;
        uint8_t send_buffer[10];
        send_buffer[ind++] = 101;  // magic nr.
        send_buffer[ind++] = 0x0;  // command ID
        send_buffer[ind++] = (uint8_t) (10 * PACKAGE_MAJOR_MINOR_VERSION);
        send_buffer[ind++] = 1;  // build number
        // Send the full type here. This is redundant with cmd_light_info. It
        // likely shouldn't be here, as the type can be reconfigured and the
        // app would need to reconnect to pick up the change from this command.
        send_buffer[ind++] = d->float_conf.hardware.leds.type;
        VESC_IF->send_app_data(send_buffer, ind);
        return;
    }
    case COMMAND_GET_RTDATA: {
        send_realtime_data(d);
        return;
    }
    case COMMAND_RT_TUNE: {
        cmd_runtime_tune(d, &buffer[2], len - 2);
        return;
    }
    case COMMAND_TUNE_OTHER: {
        if (len >= 14) {
            cmd_runtime_tune_other(d, &buffer[2], len - 2);
        } else {
            log_error("Command data length incorrect: %u", len);
        }
        return;
    }
    case COMMAND_TUNE_TILT: {
        if (len >= 10) {
            cmd_runtime_tune_tilt(d, &buffer[2], len - 2);
        } else {
            log_error("Command data length incorrect: %u", len);
        }
        return;
    }
    case COMMAND_RC_MOVE: {
        if (len == 6) {
            cmd_rc_move(d, &buffer[2]);
        } else {
            log_error("Command data length incorrect: %u", len);
        }
        return;
    }
    case COMMAND_CFG_RESTORE: {
        read_cfg_from_eeprom(&d->float_conf);
        return;
    }
    case COMMAND_TUNE_DEFAULTS: {
        cmd_tune_defaults(d);
        return;
    }
    case COMMAND_CFG_SAVE: {
        write_cfg_to_eeprom(d);
        return;
    }
    case COMMAND_PRINT_INFO: {
        cmd_print_info(d);
        return;
    }
    case COMMAND_GET_ALLDATA: {
        if (len == 3) {
            cmd_send_all_data(d, buffer[2]);
        } else {
            log_error("Command data length incorrect: %u", len);
        }
        return;
    }
    case COMMAND_EXPERIMENT: {
        cmd_experiment(d, &buffer[2]);
        return;
    }
    case COMMAND_LOCK: {
        cmd_lock(d, &buffer[2]);
        return;
    }
    case COMMAND_HANDTEST: {
        cmd_handtest(d, &buffer[2]);
        return;
    }
    case COMMAND_BOOSTER: {
        if (len == 6) {
            cmd_booster(d, &buffer[2]);
        } else {
            log_error("Command data length incorrect: %u", len);
        }
        return;
    }
    case COMMAND_FLYWHEEL: {
        if (len >= 8) {
            cmd_flywheel_toggle(d, &buffer[2], len - 2);
        } else {
            log_error("Command data length incorrect: %u", len);
        }
        return;
    }
    case COMMAND_LCM_POLL: {
        lcm_poll_request(&d->lcm, &buffer[2], len - 2);
        lcm_poll_response(&d->lcm, &d->state, d->footpad_sensor.state, &d->motor, d->pitch);
        return;
    }
    case COMMAND_LCM_LIGHT_INFO: {
        lcm_light_info_response(&d->lcm);
        return;
    }
    case COMMAND_LCM_LIGHT_CTRL: {
        lcm_light_ctrl_request(&d->lcm, &buffer[2], len - 2);
        return;
    }
    case COMMAND_LCM_DEVICE_INFO: {
        lcm_device_info_response(&d->lcm);
        return;
    }
    case COMMAND_LCM_GET_BATTERY: {
        lcm_get_battery_response(&d->lcm);
        return;
    }
    case COMMAND_CHARGING_STATE: {
        charging_state_request(&d->charging, &buffer[2], len - 2, &d->state);
        return;
    }
    case COMMAND_GET_RTDATA_2: {
        send_realtime_data2(d);
        return;
    }
    case COMMAND_LIGHTS_CONTROL: {
        lights_control_request(&d->float_conf.leds, &buffer[2], len - 2, &d->lcm);
        lights_control_response(&d->float_conf.leds);
        return;
    }
    default: {
        if (!VESC_IF->app_is_output_disabled()) {
            log_error("Unknown command received: %u", command);
        }
    }
    }
}

// Register get_debug as a lisp extension
static lbm_value ext_dbg(lbm_value *args, lbm_uint argn) {
    if (argn != 1 || !VESC_IF->lbm_is_number(args[0])) {
        return VESC_IF->lbm_enc_sym_eerror;
    }

    return VESC_IF->lbm_enc_float(app_get_debug(VESC_IF->lbm_dec_as_i32(args[0])));
}

// Called from Lisp on init to pass in the version info of the firmware
static lbm_value ext_set_fw_version(lbm_value *args, lbm_uint argn) {
    data *d = (data *) ARG;
    if (argn > 2) {
        d->fw_version_major = VESC_IF->lbm_dec_as_i32(args[0]);
        d->fw_version_minor = VESC_IF->lbm_dec_as_i32(args[1]);
        d->fw_version_beta = VESC_IF->lbm_dec_as_i32(args[2]);
    }
    return VESC_IF->lbm_enc_sym_true;
}

// Used to send the current or default configuration to VESC Tool.
static int get_cfg(uint8_t *buffer, bool is_default) {
    data *d = (data *) ARG;

    RefloatConfig *cfg;
    if (is_default) {
        cfg = VESC_IF->malloc(sizeof(RefloatConfig));
        if (!cfg) {
            log_error("Failed to send default config to VESC tool: Out of memory.");
            return 0;
        }
        confparser_set_defaults_refloatconfig(cfg);
    } else {
        cfg = &d->float_conf;
    }

    int res = confparser_serialize_refloatconfig(buffer, cfg);

    if (is_default) {
        VESC_IF->free(cfg);
    }

    return res;
}

// Used to set and write configuration from VESC Tool.
static bool set_cfg(uint8_t *buffer) {
    data *d = (data *) ARG;

    // don't let users use the Refloat Cfg "write" button in special modes
    if (d->state.mode != MODE_NORMAL) {
        return false;
    }

    bool res = confparser_deserialize_refloatconfig(buffer, &d->float_conf);

    // Store to EEPROM
    if (res) {
        write_cfg_to_eeprom(d);
        configure(d);
        leds_configure(&d->leds, &d->float_conf.leds);
    }

    return res;
}

static int get_cfg_xml(uint8_t **buffer) {
    // Note: As the address of data_refloatconfig_ is not known
    // at compile time it will be relative to where it is in the
    // linked binary. Therefore we add PROG_ADDR to it so that it
    // points to where it ends up on the STM32.
    *buffer = data_refloatconfig_ + PROG_ADDR;
    return DATA_REFLOATCONFIG__SIZE;
}

// Called when code is stopped
static void stop(void *arg) {
    data *d = (data *) arg;
    VESC_IF->imu_set_read_callback(NULL);
    VESC_IF->set_app_data_handler(NULL);
    VESC_IF->conf_custom_clear_configs();
    if (d->led_thread) {
        VESC_IF->request_terminate(d->led_thread);
    }
    if (d->main_thread) {
        VESC_IF->request_terminate(d->main_thread);
    }
    log_msg("Terminating.");
    leds_destroy(&d->leds);
    VESC_IF->free(d);
}

INIT_FUN(lib_info *info) {
    INIT_START
    log_msg("Initializing Refloat v" PACKAGE_VERSION);

    data *d = VESC_IF->malloc(sizeof(data));
    if (!d) {
        log_error("Out of memory, startup failed.");
        return false;
    }
    data_init(d);

    info->stop_fun = stop;
    info->arg = d;

    VESC_IF->conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml);

    if ((d->float_conf.is_beeper_enabled) ||
        (d->float_conf.inputtilt_remote_type != INPUTTILT_PPM)) {
        beeper_init();
    }

    balance_filter_init(&d->balance_filter);
    VESC_IF->imu_set_read_callback(imu_ref_callback);

    footpad_sensor_update(&d->footpad_sensor, &d->float_conf);

    d->main_thread = VESC_IF->spawn(refloat_thd, 1024, "Refloat Main", d);
    if (!d->main_thread) {
        log_error("Failed to spawn Refloat Main thread.");
        return false;
    }

    bool have_leds = leds_init(
        &d->leds, &d->float_conf.hardware.leds, &d->float_conf.leds, d->footpad_sensor.state
    );

    if (have_leds) {
        d->led_thread = VESC_IF->spawn(led_thd, 1024, "Refloat LEDs", d);
        if (!d->led_thread) {
            log_error("Failed to spawn Refloat LEDs thread.");
            leds_destroy(&d->leds);
        }
    }

    VESC_IF->set_app_data_handler(on_command_received);
    VESC_IF->lbm_add_extension("ext-dbg", ext_dbg);
    VESC_IF->lbm_add_extension("ext-set-fw-version", ext_set_fw_version);

    return true;
}

void send_app_data_overflow_terminate() {
    VESC_IF->request_terminate(((data *) ARG)->main_thread);
}
