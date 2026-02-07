# Command: REALTIME_DATA

**ID**: 33

Provides selectable realtime data from the package to the client. This command allows the client to request specific data fields using a bitmask, and supports encoding floats as both float16 and float32.

## Request

| Offset | Size | Name             | Description   |
|--------|------|------------------|---------------|
| 0      | 1    | `control_flags`  | Control flags specifying format options:<br> `0x1`: Use float32 instead of float16 for numeric values |
| 1      | 4    | `mask1`          | Bitmask specifying which data fields to include (bits 0-31). See **Mask1** table below. |
| 5      | 4    | `mask2`          | Bitmask specifying which data fields to include (bits 32-63). See **Mask2** table below. |

## Response

The response contains the data fields that were requested via the bitmasks. The response starts with header information, followed by the requested realtime values in the order they are listed in the mask tables.

| Offset | Size | Name             | Description   |
|--------|------|------------------|---------------|
| 0      | 1    | `control_flags`  | Echo of the control flags from the request. |
| 1      | 4    | `mask1`          | Echo of mask1 from the request. |
| 5      | 4    | `mask2`          | Echo of mask2 from the request. |
| 9      | 4    | `time`           | Timestamp of the data in ticks, as `uint32`. To convert to seconds, use `tick_rate` from the [INFO](INFO.md) command. |
| 13     | N    | `data_fields`    | Sequence of requested data fields. |

### Mask1 Fields

The following bits in `mask1` control which fields are included in the response. When a bit is set, the corresponding field is included in the response in the order listed below.

| Bit | Field Name                 | Type                     | Description                                      |
|-----|----------------------------|--------------------------|--------------------------------------------------|
| 0   | `extra_flags`              | uint8                    | Extra flags for various internal package state values. See **extra_flags** below. |
| 1   | `state_flags`              | uint32                   | Combined state, mode, footpad, and alert information. See **state_flags** below. |
| 2-5 | _(unused)_                 | -                        | Reserved for future use.                         |
| 6   | `speed`                    | float16/float32          | Current speed [km/h].                            |
| 7   | `erpm`                     | float16/float32          | Electrical RPM of the motor.                     |
| 8   | `current`                  | float16/float32          | Motor current [A].                               |
| 9   | `dir_current`              | float16/float32          | Directional motor current [A].                   |
| 10  | `filt_current`             | float16/float32          | Filtered motor current [A].                      |
| 11  | `duty_cycle`               | float16/float32          | Motor duty cycle (0.0-1.0).                      |
| 12  | `batt_voltage`             | float16/float32          | Battery voltage [V].                             |
| 13  | `batt_current`             | float16/float32          | Battery current [A].                             |
| 14  | `mosfet_temp`              | float16/float32          | MOSFET temperature [°C].                         |
| 15  | `motor_temp`               | float16/float32          | Motor temperature [°C].                          |
| 16  | `pitch`                    | float16/float32          | IMU pitch angle [°].                             |
| 17  | `balance_pitch`            | float16/float32          | Balance pitch angle [°].                         |
| 18  | `roll`                     | float16/float32          | IMU roll angle [°].                              |
| 19  | `adc1`                     | float16/float32          | Footpad sensor ADC1 value [V].                   |
| 20  | `adc2`                     | float16/float32          | Footpad sensor ADC2 value [V].                   |
| 21  | `remote_input`             | float16/float32          | Remote control input value (0.0-1.0).            |
| 22  | `setpoint`                 | float16/float32          | Current balance setpoint [°].                    |
| 23  | `atr_setpoint`             | float16/float32          | ATR setpoint [°].                                |
| 24  | `brake_tilt_setpoint`      | float16/float32          | Brake tilt setpoint [°].                         |
| 25  | `torque_tilt_setpoint`     | float16/float32          | Torque tilt setpoint [°].                        |
| 26  | `turn_tilt_setpoint`       | float16/float32          | Turn tilt setpoint [°].                          |
| 27  | `remote_setpoint`          | float16/float32          | Remote control setpoint [°].                     |
| 28  | `balance_current`          | float16/float32          | Balance (output) current [A].                    |
| 29-31 | _(unused)_               | -                        | Reserved for future use.                         |

### Mask2 Fields

The following bits in `mask2` (using `mask1` parameter but referencing bits logically in a second 32-bit space) control additional fields:

| Bit | Field Name                 | Type                     | Description                                      |
|-----|----------------------------|--------------------------|--------------------------------------------------|
| 0   | `odometer`                 | uint32                   | Total lifetime distance traveled [m].            |
| 1   | `distance_abs`             | float16/float32          | Current ride absolute distance traveled [m].     |
| 2   | `battery_level`            | float16/float32          | Battery level (0.0-1.0).                         |
| 3   | `charging_voltage`         | float16/float32          | Charging voltage [V].                            |
| 4   | `charging_current`         | float16/float32          | Charging current [A].                            |
| 5   | `amp_hours`                | float16/float32          | Amp hours consumed [Ah].                         |
| 6   | `amp_hours_charged`        | float16/float32          | Amp hours charged [Ah].                          |
| 7   | `watt_hours`               | float16/float32          | Watt hours consumed [Wh].                        |
| 8   | `watt_hours_charged`       | float16/float32          | Watt hours charged [Wh].                         |
| 9-31 | _(unused)_                | -                        | Reserved for future use.                         |

### extra_flags

| 7-3 |                      2 |                       1 |                       0 |
|-----|------------------------|-------------------------|-------------------------|
|   0 | `data_record_autostop` | `data_record_autostart` | `data_record_recording` |

The `data_record_*` flags represent data recording internal state, see [DATA_RECORD](DATA_RECORD.md).

### state_flags

The state flags are encoded as a 32-bit unsigned integer with the following bit layout:

| Bits  | Name             | Description |
|-------|------------------|-------------|
| 31-30 | (reserved)       | Reserved, always 0. |
| 29-28 | `package_mode`   | The mode of the package. |
| 27-26 | (reserved)       | Reserved, always 0. |
| 25-24 | `package_state`  | The state of the package. |
| 23-22 | `footpad_state`  | The footpad sensor state. |
| 21    | `charging`       | Whether the board is charging. |
| 20    | `fatal_error`    | Fatal error occurred (as of now only a firmware fault can cause this). |
| 19-18 | (reserved)       | Reserved, always 0. |
| 17    | `darkride`       | Whether darkride is active. |
| 16    | `wheelslip`      | Whether wheelslip is detected. |
| 15-12 | `sat`            | Setpoint Adjustment Type. |
| 11-8  | `stop_condition` | The stop condition. |
| 7-0   | `beep_reason`    | The last beep reason. |

**`package_mode`**:
- `0: NORMAL`
- `1: HANDTEST`
- `2: FLYWHEEL`

**`package_state`**:
- `0: DISABLED`
- `1: STARTUP`
- `2: READY`
- `3: RUNNING`

**`footpad_state`**:
- `0: NONE`
- `1: LEFT`
- `2: RIGHT`
- `3: BOTH`

**`sat` (setpoint adjustment type)**:
- `0: NONE`
- `1: CENTERING`
- `2: REVERSESTOP`
- `6: PB_DUTY`
- `10: PB_HIGH_VOLTAGE`
- `11: PB_LOW_VOLTAGE`
- `12: PB_TEMPERATURE`

**`stop_condition`**:
- `0: NONE`
- `1: PITCH`
- `2: ROLL`
- `3: SWITCH_HALF`
- `4: SWITCH_FULL`
- `5: REVERSE_STOP`
- `6: QUICKSTOP`

**`beep_reason`**:
- `0: NONE`
- `1: LOW_VOLTAGE`
- `2: HIGH_VOLTAGE`
- `3: TEMP_MOSFET`
- `4: TEMP_MOTOR`
- `5: CURRENT`
- `6: DUTY`
- `7: SENSORS`
- `8: LOW_BATTERY`
- `9: IDLE`
- `10: ERROR`

## Notes

- Numeric values are encoded as [float16](float16.md) by default. Set bit 0 of `control_flags` to use float32 encoding instead.
- The `odometer` field is always encoded as a uint32 regardless of the `control_flags` setting.
- Fields appear in the response in the exact order they are listed in the mask tables above.
- Only fields with their corresponding mask bit set to 1 will be included in the response.

## Example

To request only speed, current, and battery voltage with float32 precision:

**Request:**
- `control_flags`: 0x01 (use float32)
- `mask1`: 0x00001140 (bits 6, 8, 12 set for speed, current, batt_voltage)
- `mask2`: 0x00000000 (no mask2 fields)

**Response will contain:**
1. `control_flags` (1 byte): 0x01
2. `mask1` (4 bytes): 0x00001140
3. `mask2` (4 bytes): 0x00000000
4. `time` (4 bytes): current timestamp
5. `speed` (4 bytes): float32 value
6. `current` (4 bytes): float32 value
7. `batt_voltage` (4 bytes): float32 value

Total response size: 25 bytes
