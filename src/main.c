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
#include "booster.h"
#include "brake_tilt.h"
#include "charging.h"
#include "data.h"
#include "data_recorder.h"
#include "footpad_sensor.h"
#include "haptic_feedback.h"
#include "imu.h"
#include "lcm.h"
#include "leds.h"
#include "motor_control.h"
#include "motor_data.h"
#include "pid.h"
#include "remote.h"
#include "rt_data.h"
#include "state.h"
#include "time.h"
#include "torque_tilt.h"
#include "turn_tilt.h"
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

static const FootpadSensorState headlights_on_konami_sequence[] = {
    FS_LEFT, FS_NONE, FS_LEFT, FS_NONE, FS_RIGHT
};

static const FootpadSensorState headlights_off_konami_sequence[] = {
    FS_RIGHT, FS_NONE, FS_RIGHT, FS_NONE, FS_LEFT
};

static void flywheel_stop(Data *d);
static void cmd_flywheel_toggle(Data *d, unsigned char *cfg, int len);

const VESC_PIN beeper_pin = VESC_PIN_PPM;

#define REVSTOP_ERPM_INCR 0.00008
#define EXT_BEEPER_ON() VESC_IF->io_write(beeper_pin, 1)
#define EXT_BEEPER_OFF() VESC_IF->io_write(beeper_pin, 0)

void beeper_init() {
    VESC_IF->io_set_mode(beeper_pin, VESC_PIN_MODE_OUTPUT);
}

void beeper_update(Data *d) {
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

void beeper_enable(Data *d, bool enable) {
    d->beeper_enabled = enable;
    if (!enable) {
        EXT_BEEPER_OFF();
    }
}

void beep_alert(Data *d, int num_beeps, bool longbeep) {
    if (!d->beeper_enabled) {
        return;
    }
    if (d->beep_num_left == 0) {
        d->beep_num_left = num_beeps * 2 + 1;
        d->beep_duration = longbeep ? 300 : 80;
        d->beep_countdown = d->beep_duration;
    }
}

void beep_off(Data *d, bool force) {
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_OFF();
    }
}

void beep_on(Data *d, bool force) {
    if (!d->beeper_enabled) {
        return;
    }
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_ON();
    }
}

static void reconfigure(Data *d) {
    d->startup_step_size = d->float_conf.startup_speed / d->float_conf.hertz;
    d->noseangling_step_size = d->float_conf.noseangling_speed / d->float_conf.hertz;
    d->startup_pitch_trickmargin = d->float_conf.startup_dirtylandings_enabled ? 10 : 0;
    d->tiltback_variable =
        d->float_conf.tiltback_variable / 1000 * sign(d->float_conf.tiltback_variable_max);
    d->tiltback_variable_max_erpm =
        fabsf(d->float_conf.tiltback_variable_max / d->tiltback_variable);

    motor_data_configure(&d->motor, d->float_conf.atr_filter / d->float_conf.hertz);
    motor_control_configure(&d->motor_control, &d->float_conf);
    balance_filter_configure(&d->balance_filter, &d->float_conf);
    torque_tilt_configure(&d->torque_tilt, &d->float_conf);
    atr_configure(&d->atr, &d->float_conf);
    brake_tilt_configure(&d->brake_tilt, &d->float_conf);
    turn_tilt_configure(&d->turn_tilt, &d->float_conf);
    remote_configure(&d->remote, &d->float_conf);
    haptic_feedback_configure(&d->haptic_feedback, &d->float_conf);

    time_refresh_idle(&d->time);
}

static void configure(Data *d) {
    state_set_disabled(&d->state, d->float_conf.disabled);

    lcm_configure(&d->lcm, &d->float_conf.leds);

    // Loop time in microseconds
    d->loop_time_us = 1e6 / d->float_conf.hertz;

    d->tiltback_duty_step_size = d->float_conf.tiltback_duty_speed / d->float_conf.hertz;
    d->tiltback_hv_step_size = d->float_conf.tiltback_hv_speed / d->float_conf.hertz;
    d->tiltback_lv_step_size = d->float_conf.tiltback_lv_speed / d->float_conf.hertz;
    d->tiltback_return_step_size = d->float_conf.tiltback_return_speed / d->float_conf.hertz;

    // Feature: Soft Start
    d->softstart_ramp_step_size = (float) 100 / d->float_conf.hertz;

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

    d->max_duty_with_margin = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty) - 0.05;

    // Feature: Reverse Stop
    d->reverse_tolerance = 20000;
    d->reverse_stop_step_size = 100.0 / d->float_conf.hertz;

    // Speed above which to warn users about an impending full switch fault
    d->switch_warn_beep_erpm = d->float_conf.is_footbeep_enabled ? 2000 : 100000;

    d->beeper_enabled = d->float_conf.is_beeper_enabled;

    reconfigure(d);

    if (d->state.state == STATE_DISABLED) {
        beep_alert(d, 3, false);
    } else {
        beep_alert(d, 1, false);
    }
}

static void leds_headlights_switch(CfgLeds *cfg_leds, LcmData *lcm, bool headlights_on) {
    cfg_leds->headlights_on = headlights_on;
    lcm_configure(lcm, cfg_leds);
}

static void reset_runtime_vars(Data *d) {
    motor_data_reset(&d->motor);
    atr_reset(&d->atr);
    brake_tilt_reset(&d->brake_tilt);
    torque_tilt_reset(&d->torque_tilt);
    turn_tilt_reset(&d->turn_tilt);
    remote_reset(&d->remote);
    booster_reset(&d->booster);

    pid_init(&d->pid);
    d->balance_current = 0;

    // Set values for startup
    d->setpoint = d->imu.balance_pitch;
    d->setpoint_target_interpolated = d->imu.balance_pitch;
    d->setpoint_target = 0;
    d->noseangling_interpolated = 0;
    d->traction_control = false;
    d->softstart_pid_limit = 0;
    d->startup_pitch_tolerance = d->float_conf.startup_pitch_tolerance;

    // RC Move:
    d->rc_steps = 0;
    d->rc_current = 0;
}

static void engage(Data *d) {
    reset_runtime_vars(d);
    motor_control_play_click(&d->motor_control);

    state_engage(&d->state);
    timer_refresh(&d->time, &d->time.engage_timer);
    data_recorder_trigger(&d->data_record, true);
}

/**
 *  do_rc_move: perform motor movement while board is idle
 */
static void do_rc_move(Data *d) {
    if (d->rc_steps > 0) {
        d->rc_current = d->rc_current * 0.95 + d->rc_current_target * 0.05;
        if (d->motor.abs_erpm > 800) {
            d->rc_current = 0;
        }
        d->rc_steps--;
        d->rc_counter++;
        if ((d->rc_counter == 500) && (d->rc_current_target > 2)) {
            d->rc_current_target /= 2;
        }
        motor_control_request_current(&d->motor_control, d->rc_current);
    } else {
        d->rc_counter = 0;

        // Throttle must be greater than 2% (Help mitigate lingering throttle)
        if ((d->float_conf.remote_throttle_current_max > 0) &&
            time_elapsed(&d->time, disengage, d->float_conf.remote_throttle_grace_period) &&
            (fabsf(d->remote.input) > 0.02)) {
            float servo_val = d->remote.input;
            servo_val *= (d->float_conf.inputtilt_invert_throttle ? -1.0 : 1.0);
            d->rc_current = d->rc_current * 0.95 +
                (d->float_conf.remote_throttle_current_max * servo_val) * 0.05;
            motor_control_request_current(&d->motor_control, d->rc_current);
        } else {
            d->rc_current = 0;
        }
    }
}

static float get_setpoint_adjustment_step_size(Data *d) {
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

bool can_engage(const Data *d) {
    if (d->state.charging) {
        return false;
    }

    if (d->footpad.state == FS_BOTH) {
        return true;
    }

    if (d->footpad.state == FS_LEFT || d->footpad.state == FS_RIGHT) {
        // When simple start is enabled:
        // 2 seconds after stopping we allow starting with a single sensor (e.g. for jump starts)
        // Then up to 1 second after engaging only a single sensor is required even at low speed
        bool is_simple_start = d->float_conf.startup_simplestart_enabled &&
            (time_elapsed(&d->time, disengage, 2) || !time_elapsed(&d->time, engage, 1));

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
static bool check_faults(Data *d) {
    // Aggressive reverse stop in case the board runs off when upside down
    if (d->state.darkride) {
        if (d->motor.erpm > 1000) {
            // erpms are also reversed when upside down!
            if (timer_older(&d->time, d->fault_switch_timer, 0.1) || d->motor.erpm > 2000 ||
                (d->state.wheelslip && timer_older(&d->time, d->upside_down_fault_timer, 1) &&
                 timer_older(&d->time, d->fault_switch_timer, 0.03))) {

                // Trigger Reverse Stop when board is going reverse and
                // going > 2mph for more than 100ms
                // going > 4mph
                // detecting wheelslip (aka excorcist wiggle) after the first second
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
        } else {
            timer_refresh(&d->time, &d->fault_switch_timer);
            if (d->motor.erpm > 300) {
                // erpms are also reversed when upside down!
                if (timer_older(&d->time, d->fault_angle_roll_timer, 0.5)) {
                    state_stop(&d->state, STOP_REVERSE_STOP);
                    return true;
                }
            } else {
                timer_refresh(&d->time, &d->fault_angle_roll_timer);
            }
        }
        if (can_engage(d)) {
            // allow turning it off by engaging foot sensors
            state_stop(&d->state, STOP_SWITCH_HALF);
            return true;
        }
    } else {
        bool disable_switch_faults = d->float_conf.fault_moving_fault_disabled &&
            // Rolling forward (not backwards!)
            d->motor.erpm > (d->float_conf.fault_adc_half_erpm * 2) &&
            // Not tipped over
            fabsf(d->imu.roll) < 40;

        // Check switch
        // Switch fully open
        if (d->footpad.state == FS_NONE && d->state.mode != MODE_FLYWHEEL) {
            if (!disable_switch_faults) {
                if (timer_older_ms(
                        &d->time, d->fault_switch_timer, d->float_conf.fault_delay_switch_full
                    )) {
                    state_stop(&d->state, STOP_SWITCH_FULL);
                    return true;
                }
                // low speed (below 6 x half-fault threshold speed):
                else if ((d->motor.abs_erpm < d->float_conf.fault_adc_half_erpm * 6) &&
                         timer_older_ms(
                             &d->time, d->fault_switch_timer, d->float_conf.fault_delay_switch_half
                         )) {
                    state_stop(&d->state, STOP_SWITCH_FULL);
                    return true;
                }
            }

            if (d->motor.abs_erpm < 200 && fabsf(d->imu.pitch) > 14 &&
                fabsf(d->remote.setpoint) < 30 && sign(d->imu.pitch) == d->motor.erpm_sign) {
                state_stop(&d->state, STOP_QUICKSTOP);
                return true;
            }
        } else {
            timer_refresh(&d->time, &d->fault_switch_timer);
        }

        // Feature: Reverse-Stop
        if (d->state.sat == SAT_REVERSESTOP) {
            //  Taking your foot off entirely while reversing? Ignore delays
            if (d->footpad.state == FS_NONE) {
                state_stop(&d->state, STOP_SWITCH_FULL);
                return true;
            }
            if (fabsf(d->imu.pitch) > 18) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            // Above 10 degrees for a full second? Switch it off
            if (fabsf(d->imu.pitch) > 10 && timer_older(&d->time, d->reverse_timer, 1)) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            // Above 5 degrees for 2 seconds? Switch it off
            if (fabsf(d->imu.pitch) > 5 && timer_older(&d->time, d->reverse_timer, 2)) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            if (fabsf(d->reverse_total_erpm) > d->reverse_tolerance * 10) {
                state_stop(&d->state, STOP_REVERSE_STOP);
                return true;
            }
            if (fabsf(d->imu.pitch) < 5) {
                timer_refresh(&d->time, &d->reverse_timer);
            }
        }

        // Switch partially open and stopped
        if (!d->float_conf.fault_is_dual_switch) {
            if (!can_engage(d) && d->motor.abs_erpm < d->float_conf.fault_adc_half_erpm) {
                if (timer_older_ms(
                        &d->time, d->fault_switch_half_timer, d->float_conf.fault_delay_switch_half
                    )) {
                    state_stop(&d->state, STOP_SWITCH_HALF);
                    return true;
                }
            } else {
                timer_refresh(&d->time, &d->fault_switch_half_timer);
            }
        }

        // Check roll angle
        if (fabsf(d->imu.roll) > d->float_conf.fault_roll) {
            if (timer_older_ms(
                    &d->time, d->fault_angle_roll_timer, d->float_conf.fault_delay_roll
                )) {
                state_stop(&d->state, STOP_ROLL);
                return true;
            }
        } else {
            timer_refresh(&d->time, &d->fault_angle_roll_timer);

            if (d->float_conf.fault_darkride_enabled) {
                if (fabsf(d->imu.roll) > 100 && fabsf(d->imu.roll) < 135) {
                    state_stop(&d->state, STOP_ROLL);
                    return true;
                }
            }
        }

        if (d->state.mode == MODE_FLYWHEEL && d->footpad.state == FS_BOTH) {
            state_stop(&d->state, STOP_SWITCH_HALF);
            d->flywheel_abort = true;
            return true;
        }
    }

    // Check pitch angle
    if (fabsf(d->imu.pitch) > d->float_conf.fault_pitch && fabsf(d->remote.setpoint) < 30) {
        if (timer_older_ms(&d->time, d->fault_angle_pitch_timer, d->float_conf.fault_delay_pitch)) {
            state_stop(&d->state, STOP_PITCH);
            return true;
        }
    } else {
        timer_refresh(&d->time, &d->fault_angle_pitch_timer);
    }

    return false;
}

static void calculate_setpoint_target(Data *d) {
    if (d->motor.batt_voltage < d->float_conf.tiltback_hv) {
        timer_refresh(&d->time, &d->tb_highvoltage_timer);
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
                (fabsf(d->reverse_total_erpm) - d->reverse_tolerance) * REVSTOP_ERPM_INCR;
        } else {
            if (fabsf(d->reverse_total_erpm) <= d->reverse_tolerance * 0.5) {
                if (d->motor.erpm >= 0) {
                    d->state.sat = SAT_NONE;
                    d->reverse_total_erpm = 0;
                    d->setpoint_target = 0;
                    pid_reset_integral(&d->pid);
                }
            }
        }
    } else if (d->float_conf.fault_reversestop_enabled && d->motor.erpm < -200 &&
               !d->state.darkride) {
        // Detecting reverse stop takes priority over any error condition SAT
        if (d->state.sat >= SAT_PB_HIGH_VOLTAGE) {
            // If this happens while in Error-Tiltback (LV/HV/TEMP) then we need to
            // take the already existing setpoint into account
            d->reverse_total_erpm =
                -(d->reverse_tolerance + d->setpoint_target_interpolated / REVSTOP_ERPM_INCR);
        } else {
            d->reverse_total_erpm = 0;
        }
        d->state.sat = SAT_REVERSESTOP;
        timer_refresh(&d->time, &d->reverse_timer);
    } else if (d->state.mode != MODE_FLYWHEEL &&
               // not normal, either wheelslip or wheel getting stuck
               fabsf(d->motor.acceleration) > 15 &&
               sign(d->motor.acceleration) == d->motor.erpm_sign && d->motor.duty_cycle > 0.3 &&
               // acceleration can jump a lot at very low speeds
               d->motor.abs_erpm > 2000) {
        d->state.wheelslip = true;
        d->state.sat = SAT_NONE;
        timer_refresh(&d->time, &d->wheelslip_timer);
        if (d->state.darkride) {
            d->traction_control = true;
        }
    } else if (d->state.wheelslip) {
        if (fabsf(d->motor.acceleration) < 10) {
            // acceleration is slowing down, traction control seems to have worked
            d->traction_control = false;
        }
        // Remain in wheelslip state for a bit to avoid any overreactions
        if (d->motor.duty_cycle > d->max_duty_with_margin) {
            timer_refresh(&d->time, &d->wheelslip_timer);
        } else if (timer_older(&d->time, d->wheelslip_timer, 0.2)) {
            if (d->motor.duty_raw < 0.85) {
                d->traction_control = false;
                d->state.wheelslip = false;
            }
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
    } else if (d->motor.duty_cycle > 0.05 && d->motor.batt_voltage > d->float_conf.tiltback_hv) {
        d->beep_reason = BEEP_HV;
        beep_alert(d, 3, false);
        if (timer_older(&d->time, d->tb_highvoltage_timer, 0.5) ||
            d->motor.batt_voltage > d->float_conf.tiltback_hv + 1) {
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
    } else if (d->motor.mosfet_temp > d->mc_max_temp_fet) {
        // Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
        beep_alert(d, 3, true);
        d->beep_reason = BEEP_TEMPFET;
        if (d->motor.mosfet_temp > d->mc_max_temp_fet + 1) {
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
    } else if (d->motor.motor_temp > d->mc_max_temp_mot) {
        // Use the angle from Low-Voltage tiltback, but slower speed from High-Voltage tiltback
        beep_alert(d, 3, true);
        d->beep_reason = BEEP_TEMPMOT;
        if (d->motor.motor_temp > d->mc_max_temp_mot + 1) {
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
    } else if (d->motor.duty_cycle > 0.05 && d->motor.batt_voltage < d->float_conf.tiltback_lv) {
        beep_alert(d, 3, false);
        d->beep_reason = BEEP_LV;
        float abs_motor_current = fabsf(d->motor.dir_current);
        float vdelta = d->float_conf.tiltback_lv - d->motor.batt_voltage;
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
        d->state.sat = SAT_NONE;
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
            timer_refresh(&d->time, &d->upside_down_fault_timer);
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

static void apply_noseangling(Data *d) {
    // Variable Tiltback: looks at ERPM from the reference point of the set minimum ERPM
    float variable_erpm = clampf(
        d->motor.abs_erpm - d->float_conf.tiltback_variable_erpm, 0, d->tiltback_variable_max_erpm
    );
    float noseangling_target = d->tiltback_variable * variable_erpm * d->motor.erpm_sign;

    if (d->motor.abs_erpm > d->float_conf.tiltback_constant_erpm) {
        noseangling_target += d->float_conf.tiltback_constant * d->motor.erpm_sign;
    }

    rate_limitf(&d->noseangling_interpolated, noseangling_target, d->noseangling_step_size);
}

static void imu_ref_callback(float *acc, float *gyro, float *mag, float dt) {
    unused(mag);

    Data *d = (Data *) ARG;
    balance_filter_update(&d->balance_filter, gyro, acc, dt);
}

static void refloat_thd(void *arg) {
    Data *d = (Data *) arg;

    configure(d);

    while (!VESC_IF->should_terminate()) {
        time_update(&d->time, d->state.state);

        imu_update(&d->imu, &d->balance_filter, &d->state);

        beeper_update(d);

        charging_timeout(&d->charging, &d->state);

        // Darkride:
        if (d->float_conf.fault_darkride_enabled) {
            float abs_roll = fabsf(d->imu.roll);
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

        motor_data_update(&d->motor);

        remote_input(&d->remote, &d->float_conf);

        turn_tilt_aggregate(&d->turn_tilt, &d->imu);

        footpad_sensor_update(&d->footpad, &d->float_conf);

        if (d->footpad.state == FS_NONE && d->state.state == STATE_RUNNING &&
            d->state.mode != MODE_FLYWHEEL && d->motor.abs_erpm > d->switch_warn_beep_erpm) {
            // If we're at riding speed and the switch is off => ALERT the user
            // set force=true since this could indicate an imminent shutdown/nosedive
            beep_on(d, true);
            d->beep_reason = BEEP_SENSORS;
        } else {
            // if the switch comes back on we stop beeping
            beep_off(d, false);
        }

        haptic_feedback_update(
            &d->haptic_feedback, &d->motor_control, &d->state, &d->motor, &d->time
        );

        // Control Loop State Logic
        switch (d->state.state) {
        case (STATE_STARTUP):
            if (VESC_IF->imu_startup_done()) {
                reset_runtime_vars(d);
                // set state to READY so we need to meet start conditions to start
                d->state.state = STATE_READY;

                // if within 5V of LV tiltback threshold, issue 1 beep for each volt below that
                float threshold = d->float_conf.tiltback_lv + 5;
                if (d->motor.batt_voltage < threshold) {
                    int beeps = (int) fminf(6, threshold - d->motor.batt_voltage);
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
                    timer_refresh(&d->time, &d->fault_angle_pitch_timer);
                }
                motor_control_play_click(&d->motor_control);
                data_recorder_trigger(&d->data_record, false);
                break;
            }

            d->enable_upside_down = true;

            // Calculate setpoint and interpolation
            calculate_setpoint_target(d);
            rate_limitf(
                &d->setpoint_target_interpolated,
                d->setpoint_target,
                get_setpoint_adjustment_step_size(d)
            );
            d->setpoint = d->setpoint_target_interpolated;

            remote_update(&d->remote, &d->state, &d->float_conf);
            d->setpoint += d->remote.setpoint;

            if (!d->state.darkride) {
                // in case of wheelslip, don't change torque tilts, instead slightly decrease each
                // cycle
                if (d->state.wheelslip) {
                    torque_tilt_winddown(&d->torque_tilt);
                    atr_winddown(&d->atr);
                    brake_tilt_winddown(&d->brake_tilt);
                } else {
                    apply_noseangling(d);
                    d->setpoint += d->noseangling_interpolated;

                    turn_tilt_update(
                        &d->turn_tilt,
                        &d->motor,
                        &d->atr,
                        d->imu.balance_pitch,
                        d->noseangling_interpolated,
                        &d->float_conf
                    );
                    d->setpoint += d->turn_tilt.setpoint;

                    torque_tilt_update(&d->torque_tilt, &d->motor, &d->float_conf);
                    atr_update(&d->atr, &d->motor, &d->float_conf);
                    brake_tilt_update(
                        &d->brake_tilt,
                        &d->motor,
                        &d->atr,
                        &d->float_conf,
                        d->setpoint - d->imu.balance_pitch
                    );
                }

                // aggregated torque tilts:
                // if signs match between torque tilt and ATR + brake tilt, use the more significant
                // one if signs do not match, they are simply added together
                float ab_offset = d->atr.setpoint + d->brake_tilt.setpoint;
                if (sign(ab_offset) == sign(d->torque_tilt.setpoint)) {
                    d->setpoint +=
                        sign(ab_offset) * fmaxf(fabsf(ab_offset), fabsf(d->torque_tilt.setpoint));
                } else {
                    d->setpoint += ab_offset + d->torque_tilt.setpoint;
                }
            }

            pid_update(&d->pid, d->setpoint, &d->motor, &d->imu, &d->state, &d->float_conf);

            float booster_proportional = d->setpoint - d->brake_tilt.setpoint - d->imu.pitch;
            booster_update(&d->booster, &d->motor, &d->float_conf, booster_proportional);

            // Rate P and Booster are pitch-based (as opposed to balance pitch based)
            // They require to be filtered in, otherwise they'd cause a jerk
            float pitch_based = d->pid.rate_p + d->booster.current;
            if (d->softstart_pid_limit < d->motor.current_max) {
                pitch_based = fminf(fabs(pitch_based), d->softstart_pid_limit) * sign(pitch_based);
                d->softstart_pid_limit += d->softstart_ramp_step_size;
            }

            float new_current = d->pid.p + d->pid.i + pitch_based;
            float current_limit = d->motor.braking ? d->motor.current_min : d->motor.current_max;
            if (fabsf(new_current) > current_limit) {
                new_current = sign(new_current) * current_limit;
            }

            if (d->traction_control) {
                // freewheel while traction loss is detected
                d->balance_current = 0;
            } else {
                d->balance_current = d->balance_current * 0.8 + new_current * 0.2;
            }

            motor_control_request_current(&d->motor_control, d->balance_current);
            break;
        case (STATE_READY):
            if (d->state.mode == MODE_FLYWHEEL) {
                if (d->flywheel_abort || d->footpad.state == FS_BOTH) {
                    flywheel_stop(d);
                    break;
                }
            }

            if (d->state.mode != MODE_FLYWHEEL && d->imu.pitch > 75 && d->imu.pitch < 105) {
                if (konami_check(&d->flywheel_konami, &d->leds, &d->footpad, &d->time)) {
                    unsigned char enabled[6] = {0x82, 0, 0, 0, 0, 1};
                    cmd_flywheel_toggle(d, enabled, 6);
                }
            }

            if (d->float_conf.hardware.leds.mode != LED_MODE_OFF) {
                if (!d->leds.cfg->headlights_on &&
                    konami_check(&d->headlights_on_konami, &d->leds, &d->footpad, &d->time)) {
                    leds_headlights_switch(&d->float_conf.leds, &d->lcm, true);
                }

                if (d->leds.cfg->headlights_on &&
                    konami_check(&d->headlights_off_konami, &d->leds, &d->footpad, &d->time)) {
                    leds_headlights_switch(&d->float_conf.leds, &d->lcm, false);
                }
            }

            if (time_elapsed(&d->time, disengage, 10)) {
                // 10 seconds of grace period between flipping the board over and allowing darkride
                // mode
                if (d->state.darkride) {
                    beep_alert(d, 1, true);
                }
                d->enable_upside_down = false;
                d->state.darkride = false;
            }

            if (time_elapsed(&d->time, idle, 1800)) {  // alert user after 30 minutes
                if (timer_older(&d->time, d->nag_timer, 60)) {  // beep every 60 seconds
                    timer_refresh(&d->time, &d->nag_timer);
                    if (d->motor.batt_voltage > d->idle_voltage) {
                        // don't beep if the voltage keeps increasing (board is charging)
                        d->idle_voltage = d->motor.batt_voltage;
                    } else {
                        d->beep_reason = BEEP_IDLE;
                        beep_alert(d, 2, 1);
                    }
                }
            } else {
                timer_refresh(&d->time, &d->nag_timer);
                d->idle_voltage = 0;
            }

            if (timer_older(&d->time, d->fault_angle_pitch_timer, 1)) {
                // 1 second after disengaging - set startup tolerance back to normal (aka tighter)
                d->startup_pitch_tolerance = d->float_conf.startup_pitch_tolerance;
            }

            // Check for valid startup position and switch state
            if (fabsf(d->imu.balance_pitch) < d->startup_pitch_tolerance &&
                fabsf(d->imu.roll) < d->float_conf.startup_roll_tolerance && can_engage(d)) {
                engage(d);
                break;
            }
            // Ignore roll for the first second while it's upside down
            if (d->state.darkride && (fabsf(d->imu.balance_pitch) < d->startup_pitch_tolerance)) {
                if (time_elapsed(&d->time, disengage, 1)) {
                    // after 1 second:
                    if (fabsf(fabsf(d->imu.roll) - 180) < d->float_conf.startup_roll_tolerance) {
                        engage(d);
                        break;
                    }
                } else {
                    // 1 second or less, ignore roll-faults to allow for kick flips etc.
                    if (d->state.stop_condition != STOP_REVERSE_STOP) {
                        // don't instantly re-engage after a reverse fault!
                        engage(d);
                        break;
                    }
                }
            }
            // Push-start aka dirty landing Part II
            if (d->float_conf.startup_pushstart_enabled && d->motor.abs_erpm > 1000 &&
                can_engage(d)) {
                if ((fabsf(d->imu.balance_pitch) < 45) && (fabsf(d->imu.roll) < 45)) {
                    // 45 to prevent board engaging when upright or laying sideways
                    // 45 degree tolerance is more than plenty for tricks / extreme mounts
                    engage(d);
                    break;
                }
            }

            do_rc_move(d);
            break;
        case (STATE_DISABLED):
            break;
        }

        motor_control_apply(&d->motor_control, d->motor.abs_erpm_smooth, d->state.state, &d->time);

        data_recorder_sample(&d->data_record, d);

        VESC_IF->sleep_us(d->loop_time_us);
    }
}

static void write_cfg_to_eeprom(Data *d) {
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

        beep_alert(d, 1, 0);
        leds_status_confirm(&d->leds);
    } else {
        log_error("Failed to write config to EEPROM.");
    }
}

static void aux_thd(void *arg) {
    Data *d = (Data *) arg;

    while (!VESC_IF->should_terminate()) {
        leds_update(&d->leds, &d->state, d->footpad.state);

        // store odometer if we've gone more than 200m
        if (d->state.state != STATE_RUNNING && VESC_IF->mc_get_odometer() > d->odometer + 200) {
            VESC_IF->store_backup_data();
            d->odometer = VESC_IF->mc_get_odometer();
        }

        VESC_IF->sleep_us(1e6 / LEDS_REFRESH_RATE);
    }
}

static void read_cfg_from_eeprom(Data *d) {
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
            confparser_set_defaults_refloatconfig(&d->float_conf);
            return;
        }
    }

    if (read_ok) {
        memcpy(&d->float_conf, buffer, sizeof(RefloatConfig));
    } else {
        log_error("Failed to read config from EEPROM, using defaults.");
        confparser_set_defaults_refloatconfig(&d->float_conf);
    }

    VESC_IF->free(buffer);
}

static void data_init(Data *d) {
    memset(d, 0, sizeof(Data));

    read_cfg_from_eeprom(d);

    d->odometer = VESC_IF->mc_get_odometer();

    balance_filter_init(&d->balance_filter);
    state_init(&d->state);
    time_init(&d->time);
    pid_init(&d->pid);
    motor_control_init(&d->motor_control);
    haptic_feedback_init(&d->haptic_feedback);
    lcm_init(&d->lcm, &d->float_conf.hardware.leds);
    charging_init(&d->charging);
    remote_init(&d->remote);
    leds_init(&d->leds);
    data_recorder_init(&d->data_record);

    konami_init(&d->flywheel_konami, flywheel_konami_sequence, sizeof(flywheel_konami_sequence));
    konami_init(
        &d->headlights_on_konami,
        headlights_on_konami_sequence,
        sizeof(headlights_on_konami_sequence)
    );
    konami_init(
        &d->headlights_off_konami,
        headlights_off_konami_sequence,
        sizeof(headlights_off_konami_sequence)
    );
}

static float app_get_debug(int index) {
    Data *d = (Data *) ARG;

    switch (index) {
    case (1):
        return d->balance_current;
    case (2):
        return d->pid.p;
    case (3):
        return d->pid.i;
    case (4):
        return d->pid.rate_p;
    case (5):
        return d->setpoint;
    case (6):
        return d->atr.setpoint;
    case (7):
        return d->motor.erpm;
    case (8):
        return d->motor.dir_current;
    case (9):
        return d->motor.filt_current;
    default:
        return 0;
    }
}

// See also:
// LcmCommands in lcm.h
// ChargingCommands in charging.h
enum {
    COMMAND_INFO = 0,  // get version / package info
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
    COMMAND_REALTIME_DATA = 31,
    COMMAND_REALTIME_DATA_IDS = 32,
    COMMAND_DATA_RECORD_REQUEST = 41,

    // commands above 200 are unstable and can change protocol at any time
    COMMAND_LIGHTS_CONTROL = 202,
} Commands;

static void send_realtime_data(Data *d) {
    static const int bufsize = 72;
    uint8_t buffer[bufsize];
    int32_t ind = 0;
    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_GET_RTDATA;

    // RT Data
    buffer_append_float32_auto(buffer, d->balance_current, &ind);
    buffer_append_float32_auto(buffer, d->imu.balance_pitch, &ind);
    buffer_append_float32_auto(buffer, d->imu.roll, &ind);

    uint8_t state = (state_compat(&d->state) & 0xF);
    buffer[ind++] = (state & 0xF) + (sat_compat(&d->state) << 4);
    state = footpad_sensor_state_to_switch_compat(d->footpad.state);
    if (d->state.mode == MODE_HANDTEST) {
        state |= 0x8;
    }
    buffer[ind++] = (state & 0xF) + (d->beep_reason << 4);
    buffer_append_float32_auto(buffer, d->footpad.adc1, &ind);
    buffer_append_float32_auto(buffer, d->footpad.adc2, &ind);

    // Setpoints
    buffer_append_float32_auto(buffer, d->setpoint, &ind);
    buffer_append_float32_auto(buffer, d->atr.setpoint, &ind);
    buffer_append_float32_auto(buffer, d->brake_tilt.setpoint, &ind);
    buffer_append_float32_auto(buffer, d->torque_tilt.setpoint, &ind);
    buffer_append_float32_auto(buffer, d->turn_tilt.setpoint, &ind);
    buffer_append_float32_auto(buffer, d->remote.setpoint, &ind);

    // DEBUG
    buffer_append_float32_auto(buffer, d->imu.pitch, &ind);
    buffer_append_float32_auto(buffer, d->motor.filt_current, &ind);
    buffer_append_float32_auto(buffer, d->atr.accel_diff, &ind);
    if (d->state.charging) {
        buffer_append_float32_auto(buffer, d->charging.current, &ind);
        buffer_append_float32_auto(buffer, d->charging.voltage, &ind);
    } else {
        buffer_append_float32_auto(buffer, d->booster.current, &ind);
        buffer_append_float32_auto(buffer, d->motor.dir_current, &ind);
    }
    buffer_append_float32_auto(buffer, d->remote.input, &ind);

    SEND_APP_DATA(buffer, bufsize, ind);
}

static void cmd_send_all_data(Data *d, unsigned char mode) {
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
        buffer_append_float16(buffer, d->balance_current, 10, &ind);
        buffer_append_float16(buffer, d->imu.balance_pitch, 10, &ind);
        buffer_append_float16(buffer, d->imu.roll, 10, &ind);

        uint8_t state = (state_compat(&d->state) & 0xF) + (sat_compat(&d->state) << 4);
        buffer[ind++] = state;

        // passed switch-state includes bit3 for handtest, and bits4..7 for beep reason
        state = footpad_sensor_state_to_switch_compat(d->footpad.state);
        if (d->state.mode == MODE_HANDTEST) {
            state |= 0x8;
        }
        buffer[ind++] = (state & 0xF) + (d->beep_reason << 4);
        d->beep_reason = BEEP_NONE;

        buffer[ind++] = d->footpad.adc1 * 50;
        buffer[ind++] = d->footpad.adc2 * 50;

        // Setpoints (can be positive or negative)
        buffer[ind++] = d->setpoint * 5 + 128;
        buffer[ind++] = d->atr.setpoint * 5 + 128;
        buffer[ind++] = d->brake_tilt.setpoint * 5 + 128;
        buffer[ind++] = d->torque_tilt.setpoint * 5 + 128;
        buffer[ind++] = d->turn_tilt.setpoint * 5 + 128;
        buffer[ind++] = d->remote.setpoint * 5 + 128;

        buffer_append_float16(buffer, d->imu.pitch, 10, &ind);
        buffer[ind++] = d->booster.current + 128;

        // Now send motor stuff:
        buffer_append_float16(buffer, d->motor.batt_voltage, 10, &ind);
        buffer_append_int16(buffer, d->motor.erpm, &ind);
        buffer_append_float16(buffer, d->motor.speed * (1.0f / 3.6f), 10, &ind);
        buffer_append_float16(buffer, d->motor.current, 10, &ind);
        buffer_append_float16(buffer, d->motor.batt_current, 10, &ind);
        buffer[ind++] = d->motor.duty_raw * 100 + 128;
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
            buffer[ind++] = fmaxf(0, d->motor.mosfet_temp * 2);
            buffer[ind++] = fmaxf(0, d->motor.motor_temp * 2);
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
            buffer[ind++] = fmaxf(0, fminf(125, VESC_IF->mc_get_battery_level(NULL) * 100)) * 2;
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

static void cmd_print_info(Data *d) {
    unused(d);
}

static void cmd_lock(Data *d, unsigned char *cfg) {
    if (d->state.state != STATE_RUNNING) {
        // restore config before locking to avoid accidentally writing temporary changes
        read_cfg_from_eeprom(d);
        d->float_conf.disabled = cfg[0];
        state_set_disabled(&d->state, cfg[0]);
        write_cfg_to_eeprom(d);
    }
}

static void cmd_handtest(Data *d, unsigned char *cfg) {
    if (d->state.state != STATE_READY) {
        return;
    }

    if (d->state.mode != MODE_NORMAL && d->state.mode != MODE_HANDTEST) {
        return;
    }

    d->state.mode = cfg[0] ? MODE_HANDTEST : MODE_NORMAL;
    if (d->state.mode == MODE_HANDTEST) {
        // temporarily reduce max currents to make hand test safer / gentler
        d->motor.current_max = d->motor.current_min = 7;
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
        read_cfg_from_eeprom(d);
        configure(d);
    }
}

static void cmd_experiment(Data *d, unsigned char *cfg) {
    unused(d);
    unused(cfg);
}

static void cmd_booster(Data *d, unsigned char *cfg) {
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
static void cmd_runtime_tune(Data *d, unsigned char *cfg, int len) {
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
        d->motor.current_max = h1 * 5 + 55;
        d->motor.current_min = h2 * 5 + 55;
        if (h1 == 0) {
            d->motor.current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
        }
        if (h2 == 0) {
            d->motor.current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
        }
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

static void cmd_tune_defaults(Data *d) {
    d->float_conf.kp = CFG_DFLT_KP;
    d->float_conf.kp2 = CFG_DFLT_KP2;
    d->float_conf.ki = CFG_DFLT_KI;
    d->float_conf.mahony_kp = CFG_DFLT_MAHONY_KP;
    d->float_conf.mahony_kp_roll = CFG_DFLT_MAHONY_KP_ROLL;
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

    reconfigure(d);
}

/**
 * cmd_runtime_tune_tilt: Extract settings from 20byte message but don't write to EEPROM!
 */
static void cmd_runtime_tune_tilt(Data *d, unsigned char *cfg, int len) {
    unused(len);
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

    beep_alert(d, 3, 0);
}

/**
 * cmd_runtime_tune_other		Extract settings from 20byte message but don't write to
 * EEPROM!
 */
static void cmd_runtime_tune_other(Data *d, unsigned char *cfg, int len) {
    unsigned int flags = cfg[0];
    d->beeper_enabled = ((flags & 0x2) == 2);
    d->float_conf.fault_reversestop_enabled = ((flags & 0x4) == 4);
    d->float_conf.fault_is_dual_switch = ((flags & 0x8) == 8);
    d->float_conf.fault_darkride_enabled = ((flags & 0x10) == 0x10);
    bool dirty_landings = ((flags & 0x20) == 0x20);
    d->float_conf.startup_simplestart_enabled = ((flags & 0x40) == 0x40);
    d->float_conf.startup_pushstart_enabled = ((flags & 0x80) == 0x80);

    d->float_conf.is_beeper_enabled = d->beeper_enabled;
    d->float_conf.startup_dirtylandings_enabled = dirty_landings;

    // startup
    float ctrspeed = cfg[1];
    float pitchtolerance = cfg[2];
    float rolltolerance = cfg[3];
    float brakecurrent = cfg[4];
    float clickcurrent = cfg[5];

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
            d->float_conf.noseangling_speed = tiltspeed / 10;
        }
        d->float_conf.tiltback_variable = tiltvarrate / 100;
        d->float_conf.tiltback_variable_max = tiltvarmax / 10;
        d->float_conf.tiltback_variable_erpm = cfg[11] * 100;
    }

    if (len >= 14) {
        int inputtilt = cfg[12] & 0x3;
        if (inputtilt <= INPUTTILT_PPM) {
            d->float_conf.inputtilt_remote_type = inputtilt;
            if (inputtilt > 0) {
                d->float_conf.inputtilt_angle_limit = cfg[12] >> 2;
                d->float_conf.inputtilt_speed = cfg[13];
            }
        }
    }

    reconfigure(d);
}

void cmd_rc_move(Data *d, unsigned char *cfg) {
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

static void cmd_flywheel_toggle(Data *d, unsigned char *cfg, int len) {
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
        if (d->imu.flywheel_pitch_offset == 0 || command == 2) {
            // accidental button press?? board isn't evn close to being upright
            if (fabsf(d->imu.pitch) < 70) {
                d->state.mode = MODE_NORMAL;
                return;
            }

            imu_set_flywheel_offsets(&d->imu);
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

        // Aggressive P with some D (aka Rate-P) for Mahony kp=0.3
        d->float_conf.kp = 8.0;
        d->float_conf.kp2 = 0.3;

        if (cfg[1] > 0) {
            d->float_conf.kp = cfg[1] * 0.1f;
        }
        if (cfg[2] > 0) {
            d->float_conf.kp2 = cfg[2] * 0.01f;
        }

        d->float_conf.tiltback_duty_angle = 2;
        d->float_conf.tiltback_duty = 0.1;
        d->float_conf.tiltback_duty_speed = 5;
        d->float_conf.tiltback_return_speed = 5;

        if (cfg[3] > 0) {
            d->float_conf.tiltback_duty_angle = cfg[3] * 0.1f;
        }
        if (cfg[4] > 0) {
            d->float_conf.tiltback_duty = cfg[4] * 0.01f;
        }
        if ((len > 6) && (cfg[6] > 1) && (cfg[6] < 100)) {
            d->float_conf.tiltback_duty_speed = cfg[6] * 0.5f;
            d->float_conf.tiltback_return_speed = cfg[6] * 0.5f;
        }
        d->tiltback_duty_step_size = d->float_conf.tiltback_duty_speed / d->float_conf.hertz;
        d->tiltback_return_step_size = d->float_conf.tiltback_return_speed / d->float_conf.hertz;

        // Limit amps
        d->motor.current_max = d->motor.current_min = 40;

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

void flywheel_stop(Data *d) {
    beep_on(d, 1);
    d->state.mode = MODE_NORMAL;
    read_cfg_from_eeprom(d);
    configure(d);
}

static void cmd_realtime_data_ids() {
    static const int bufsize = 2 + 2 + ITEMS_IDS_SIZE(RT_DATA_ALL_ITEMS);
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_REALTIME_DATA_IDS;

#define ADD_ID(id) buffer_append_string(buffer, #id, &ind);
    // Send string ids of the realtime data items. The format is:
    // IDS_COUNT (1B)
    // [
    //   ID_LENTGTH (1B)
    //   [ID_CHARS] (number of chars equal to ID_LENGTH, terminal null omitted)
    // ] (ids one after another, times IDS_COUNT)
    //
    // The pattern is repeated twice, the first set of ids is realtime data
    // that is always sent, the second set of ids is realtime _runtime_
    // data, only sent when the board is engaged.
    buffer[ind++] = ITEMS_COUNT(RT_DATA_ITEMS);
    VISIT(RT_DATA_ITEMS, ADD_ID);
    buffer[ind++] = ITEMS_COUNT(RT_DATA_RUNTIME_ITEMS);
    VISIT(RT_DATA_RUNTIME_ITEMS, ADD_ID);
#undef ADD_ID

    SEND_APP_DATA(buffer, bufsize, ind);
}

static void cmd_realtime_data(Data *d) {
    static const int bufsize = 79;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_REALTIME_DATA;

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

    const DataRecord *rd = &d->data_record;
    uint8_t extra_flags = rd->autostop << 2 | rd->autostart << 1 | rd->recording;
    buffer[ind++] = extra_flags;

    buffer_append_uint32(buffer, d->time.now, &ind);

    buffer[ind++] = d->state.mode << 4 | d->state.state;

    uint8_t flags = d->state.charging << 5 | d->state.darkride << 1 | d->state.wheelslip;
    buffer[ind++] = d->footpad.state << 6 | flags;

    buffer[ind++] = d->state.sat << 4 | d->state.stop_condition;

    buffer[ind++] = d->beep_reason;

#define WRITE_VALUE(id) buffer_append_float16_auto(buffer, d->id, &ind);
    VISIT(RT_DATA_ITEMS, WRITE_VALUE);

    if (d->state.state == STATE_RUNNING) {
        VISIT(RT_DATA_RUNTIME_ITEMS, WRITE_VALUE);
    }
#undef WRITE_VALUE

    if (d->state.charging) {
        buffer_append_float16_auto(buffer, d->charging.current, &ind);
        buffer_append_float16_auto(buffer, d->charging.voltage, &ind);
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

        lcm_configure(lcm, leds);
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

static void cmd_info(const Data *d, unsigned char *buf, int len) {
    static const int bufsize = 4 + 20 + 3 + 20 + 13;
    uint8_t version = 1;
    int32_t i = 0;

    if (len > 0) {
        version = buf[i++];
    }

    int32_t ind = 0;
    uint8_t send_buffer[bufsize];
    send_buffer[ind++] = 101;  // Package ID
    send_buffer[ind++] = COMMAND_INFO;

    switch (version) {
    case 1:
        send_buffer[ind++] = MAJOR_VERSION * 10 + MINOR_VERSION;
        send_buffer[ind++] = 1;  // build number

        // Backwards compatibility for the LED type - external used to be 3
        uint8_t led_type = d->float_conf.hardware.leds.mode;
        if (led_type == LED_MODE_EXTERNAL) {
            led_type = 3;
        }

        // Send the full type here. This is redundant with cmd_light_info. It
        // likely shouldn't be here, as the type can be reconfigured and the
        // app would need to reconnect to pick up the change from this command.
        send_buffer[ind++] = led_type;
        break;
    case 2:
    // in case of unknown version, respond with the highest one that we know
    default: {
        uint8_t flags = 0;
        if (version == 2 && len > 1) {
            flags = buf[i++];
        }

        send_buffer[ind++] = 2;  // actual COMMAND_INFO version
        send_buffer[ind++] = flags;  // the flags repeated in the response

        buffer_append_string_fixed(send_buffer, PACKAGE_NAME, &ind, 20);

        send_buffer[ind++] = MAJOR_VERSION;
        send_buffer[ind++] = MINOR_VERSION;
        send_buffer[ind++] = PATCH_VERSION;

        buffer_append_string_fixed(send_buffer, VERSION_SUFFIX, &ind, 20);
        buffer_append_uint32(send_buffer, GIT_HASH, &ind);

        buffer_append_uint32(send_buffer, SYSTEM_TICK_RATE_HZ, &ind);
        uint32_t capabilities = 0;
        if (data_recorder_has_capability(&d->data_record)) {
            capabilities |= 1 << 31;
        }
        if (d->float_conf.hardware.leds.mode != LED_MODE_OFF) {
            capabilities |= 1;
            if (d->float_conf.hardware.leds.mode == LED_MODE_EXTERNAL) {
                capabilities |= 1 << 1;
            }
        }

        buffer_append_uint32(send_buffer, capabilities, &ind);

        uint8_t extra_flags = 0;
        send_buffer[ind++] = extra_flags;
    }
    }

    SEND_APP_DATA(send_buffer, bufsize, ind);
}

// Handler for incoming app commands
static void on_command_received(unsigned char *buffer, unsigned int len) {
    Data *d = (Data *) ARG;
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
    case COMMAND_INFO: {
        cmd_info(d, &buffer[2], len - 2);
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
        if (len >= 7) {
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
        read_cfg_from_eeprom(d);
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
        lcm_poll_response(&d->lcm, &d->state, d->footpad.state, &d->motor, d->imu.pitch);
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
    case COMMAND_REALTIME_DATA: {
        cmd_realtime_data(d);
        return;
    }
    case COMMAND_REALTIME_DATA_IDS: {
        cmd_realtime_data_ids();
        return;
    }
    case COMMAND_LIGHTS_CONTROL: {
        lights_control_request(&d->float_conf.leds, &buffer[2], len - 2, &d->lcm);
        lights_control_response(&d->float_conf.leds);
        return;
    }
    case COMMAND_DATA_RECORD_REQUEST: {
        data_recorder_request(&d->data_record, &buffer[2], len - 2);
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
    Data *d = (Data *) ARG;
    if (argn > 2) {
        d->fw_version_major = VESC_IF->lbm_dec_as_i32(args[0]);
        d->fw_version_minor = VESC_IF->lbm_dec_as_i32(args[1]);
        d->fw_version_beta = VESC_IF->lbm_dec_as_i32(args[2]);
    }
    return VESC_IF->lbm_enc_sym_true;
}

// Used to send the current or default configuration to VESC Tool.
static int get_cfg(uint8_t *buffer, bool is_default) {
    Data *d = (Data *) ARG;

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
    Data *d = (Data *) ARG;

    // don't let users use the Refloat Cfg "write" button in special modes
    if (d->state.mode != MODE_NORMAL) {
        return false;
    }

    bool res = confparser_deserialize_refloatconfig(buffer, &d->float_conf);

    // don't allow to disable the package in the RUNNING state
    if (d->state.state == STATE_RUNNING) {
        d->float_conf.disabled = false;
    }

    // Always reset the is_default flag on writing - whatever we write we
    // consider to not be the default config anymore
    d->float_conf.meta.is_default = false;

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
    Data *d = (Data *) arg;
    VESC_IF->imu_set_read_callback(NULL);
    VESC_IF->set_app_data_handler(NULL);
    VESC_IF->conf_custom_clear_configs();
    if (d->aux_thread) {
        VESC_IF->request_terminate(d->aux_thread);
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
    log_msg("Initializing " PACKAGE_NAME " " PACKAGE_VERSION " (%x)", GIT_HASH);

    Data *d = VESC_IF->malloc(sizeof(Data));
    if (!d) {
        log_error("Out of memory, startup failed.");
        return false;
    }
    data_init(d);

    info->stop_fun = stop;
    info->arg = d;

    if ((d->float_conf.is_beeper_enabled) ||
        (d->float_conf.inputtilt_remote_type != INPUTTILT_PPM)) {
        beeper_init();
    }

    d->main_thread = VESC_IF->spawn(refloat_thd, 1536, "Refloat Main", d);
    if (!d->main_thread) {
        log_error("Failed to spawn Refloat Main thread.");
        return false;
    }

    d->aux_thread = VESC_IF->spawn(aux_thd, 1024, "Refloat Aux", d);
    if (!d->aux_thread) {
        log_error("Failed to spawn Refloat Auxiliary thread.");
        VESC_IF->request_terminate(d->main_thread);
        return false;
    }

    footpad_sensor_update(&d->footpad, &d->float_conf);
    leds_setup(&d->leds, &d->float_conf.hardware.leds, &d->float_conf.leds, d->footpad.state);

    VESC_IF->imu_set_read_callback(imu_ref_callback);
    VESC_IF->conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml);
    VESC_IF->set_app_data_handler(on_command_received);
    VESC_IF->lbm_add_extension("ext-dbg", ext_dbg);
    VESC_IF->lbm_add_extension("ext-set-fw-version", ext_set_fw_version);

    return true;
}

void send_app_data_overflow_terminate() {
    stop(ARG);
}
