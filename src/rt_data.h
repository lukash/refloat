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

// List of items to send via the REALTIME_DATA command.
//
// The listed items directly reference members of the Data struct.
//
// RT_DATA_ITEMS are always sent, RT_DATA_RUNTIME_ITEMS are sent only when Running.
//
// Caution when changing or removing existing items! Their IDs are part of the
// Command interface (an inherent drawback of the design, a price for having a
// single easy-to-modify list). Renaming or removing an item here may make a
// client app miss the item and potentially misbehave (the package AppUI
// handles this gracefully, but the value may still be missing if it's used in
// the UI).

#define RT_DATA_ITEMS(I)                                                                           \
    I(motor.speed)                                                                                 \
    I(motor.erpm)                                                                                  \
    I(motor.current)                                                                               \
    I(motor.dir_current)                                                                           \
    I(motor.filt_current)                                                                          \
    I(motor.duty_cycle)                                                                            \
    I(motor.batt_voltage)                                                                          \
    I(motor.batt_current)                                                                          \
    I(motor.mosfet_temp)                                                                           \
    I(motor.motor_temp)                                                                            \
    I(imu.pitch)                                                                                   \
    I(imu.balance_pitch)                                                                           \
    I(imu.roll)                                                                                    \
    I(footpad.adc1)                                                                                \
    I(footpad.adc2)                                                                                \
    I(remote.input)

#define RT_DATA_RUNTIME_ITEMS(I)                                                                   \
    I(setpoint)                                                                                    \
    I(atr.setpoint)                                                                                \
    I(brake_tilt.setpoint)                                                                         \
    I(torque_tilt.setpoint)                                                                        \
    I(turn_tilt.setpoint)                                                                          \
    I(remote.setpoint)                                                                             \
    I(balance_current)                                                                             \
    I(atr.accel_diff)                                                                              \
    I(atr.speed_boost)                                                                             \
    I(booster.current)

#define VISIT(LIST, ACTION) LIST(ACTION)

#define __COUNT_ITEMS(id) +1
#define ITEMS_COUNT(LIST) (VISIT(LIST, __COUNT_ITEMS))

#define __COUNT_IDS_SIZE(id) +sizeof(#id)
#define ITEMS_IDS_SIZE(LIST) (VISIT(LIST, __COUNT_IDS_SIZE))
