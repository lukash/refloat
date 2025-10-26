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

#pragma once

// List of items to send via the REALTIME_DATA command and record in the data
// recorder.
//
// S: Send in realtime data
// R: Record the data in data recorder in addition to sending in realtime data
//
// RT_DATA_ITEMS are always sent, RT_DATA_RUNTIME_ITEMS are sent only when Running.
// Items from both lists are recorded all the time (when recording is on).
//
// The listed items directly reference members of the Data struct.
//
// Caution when changing or removing existing items! Their IDs are part of the
// Command interface (an inherent drawback of the design, a price for having a
// single easy-to-modify list). Renaming or removing an item here may make a
// client app miss the item and potentially misbehave (the package AppUI
// handles this gracefully, but the value may still be missing if it's used in
// the UI).

#define RT_DATA_ITEMS(S, R)                                                                        \
    S(motor.speed, "speed")                                                                        \
    R(motor.erpm, "erpm")                                                                          \
    S(motor.current, "current")                                                                    \
    R(motor.dir_current, "dir_current")                                                            \
    S(motor.filt_current.value, "filt_current")                                                    \
    R(motor.duty_cycle, "duty_cycle")                                                              \
    R(motor.batt_voltage, "batt_voltage")                                                          \
    S(motor.batt_current, "batt_current")                                                          \
    S(motor.mosfet_temp, "mosfet_temp")                                                            \
    S(motor.motor_temp, "motor_temp")                                                              \
    R(imu.pitch, "pitch")                                                                          \
    R(imu.balance_pitch, "balance_pitch")                                                          \
    S(imu.roll, "roll")                                                                            \
    S(footpad.adc1, "adc1")                                                                        \
    S(footpad.adc2, "adc2")                                                                        \
    S(remote.input, "remote.input")

#define RT_DATA_RUNTIME_ITEMS(S, R)                                                                \
    R(setpoint, "setpoint")                                                                        \
    R(atr.setpoint, "atr.setpoint")                                                                \
    S(brake_tilt.setpoint, "brake_tilt.setpoint")                                                  \
    R(torque_tilt.setpoint, "torque_tilt.setpoint")                                                \
    S(turn_tilt.setpoint, "turn_tilt.setpoint")                                                    \
    S(remote.setpoint, "remote.setpoint")                                                          \
    R(balance_current, "balance_current")                                                          \
    S(atr.accel_diff, "atr.accel_diff")                                                            \
    S(atr.speed_boost, "atr.speed_boost")                                                          \
    S(booster.current, "booster.current")

#define RT_DATA_ALL_ITEMS(S, R) RT_DATA_ITEMS(S, R) RT_DATA_RUNTIME_ITEMS(S, R)

#define __NOOP(target, id)

#define VISIT(LIST, ACTION) LIST(ACTION, ACTION)
#define VISIT_REC(LIST, ACTION) LIST(__NOOP, ACTION)

#define __COUNT_ITEMS(target, id) +1
#define __COUNT_IDS_SIZE(target, id) +sizeof(id)

#define ITEMS_COUNT(LIST) (VISIT(LIST, __COUNT_ITEMS))
#define ITEMS_IDS_SIZE(LIST) (VISIT(LIST, __COUNT_IDS_SIZE))

#define ITEMS_COUNT_REC(LIST) (VISIT_REC(LIST, __COUNT_ITEMS))
#define ITEMS_IDS_SIZE_REC(LIST) (VISIT_REC(LIST, __COUNT_IDS_SIZE))
