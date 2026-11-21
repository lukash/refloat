# Command: RT_TUNE

**ID**: 2

Sets tune values at runtime, possibly also while riding. Values are applied immediately and kept in RAM only; they are not written to EEPROM.

This is a one-way command with no response.

## Request

The request has grown over time by appending to it. The package reads each part only if the request is long enough to contain it, so shorter requests of older apps keep working. Additional bytes at the end are ignored, so a longer request also works on an older package, which applies the parts it knows.

| Offset | Size | Minimum request length | Description |
|--------|------|------------------------|-------------|
| 0      | 12   | 12 | [Coarse values](#coarse-values) |
| 12     | 4    | 16 | [Coarse values](#coarse-values), continued |
| 16     | 1    | 17 | [Coarse values](#coarse-values), continued |
| 17     | 2    | 19 | [Coarse values](#coarse-values), continued |
| 19     | 7    | 26 | [Time constants](#time-constants) |
| 26     | 68   | 94 | [Full resolution values](#full-resolution-values) |

Parts that are missing leave their fields unchanged. A part is only applied as a whole.

When the length is at least 17, the board beeps once (if the beeper is enabled).

### Coarse Values

Each byte holds two 4-bit values. The formulas show how to encode a field's value. The decoded values are not clamped.

| Offset | Low 4 bits | High 4 bits |
|--------|------------|-------------|
| 0      | `kp`: value − 15 | `kp2`: value × 10 |
| 1      | `ki`: 0 = 0, 1 = 0.005, 2 to 15 = 0.01 to 0.14 | `ki_limit`: 0 = 0, otherwise value − 19 |
| 2      | `booster_angle`: value − 5 | `booster_ramp`: value − 2 |
| 3      | `booster_current`: 0 = 0, otherwise (value − 8) / 2 | `turntilt_strength`: value |
| 4      | `turntilt_angle_limit` (bits 0–1), `turntilt_start_erpm` (bits 2–3): value − 2; (value − 1000) / 500 | `mahony_kp`: (value − 1.5) × 10 |
| 5      | `atr_strength_up`: 0 = 0, otherwise (value − 0.5) × 10 | `atr_strength_down`: 0 = 0, otherwise (value − 0.5) × 10 |
| 6      | `atr_speed_boost` sign: 0 = positive, otherwise negative | `atr_speed_boost`: absolute value × 20 |
| 7      | `atr_angle_limit`: value − 5 | `atr.filter.on_speed_limit` (bits 4–5), `atr.filter.off_speed_limit` (bits 6–7): value − 3; value − 2. Both ignored if all four bits are 0 |
| 8      | unused | `atr.transition_boost`: (value − 1) × 5 |
| 9      | `atr_amps_accel_ratio`: value − 5 | `atr_amps_decel_ratio`: value − 5 |
| 10     | `braketilt_strength`: value | `braketilt_lingering`: value |
| 11     | unused | unused |
| 12     | `atr_threshold_up`: value × 2 | `atr_threshold_down`: value × 2 |
| 13     | `torquetilt_strength`: value / 0.03 | `torquetilt_strength_regen`: value / 0.03 |
| 14     | `torquetilt_angle_limit`: value × 2 | `torquetilt_start_current`: value − 15 |
| 15     | `torque_tilt.filter.on_speed_limit`: value − 3 (if the length is at least 19, otherwise value × 2) | `torque_tilt.filter.off_speed_limit`: value − 3 |
| 16     | `kp_brake`: value × 10 − 1 | `kp2_brake`: value × 10 |
| 17     | `mahony_kp_roll`: (value − 1) × 10, 0 = unchanged | `turntilt_start_angle`: value, 0 = unchanged |
| 18     | `atr.filter.on_speed_limit`: value / 2 | `atr.filter.off_speed_limit`: value / 2. Both unchanged if either is 0 |

The 4-bit values limit the range and resolution of these fields. With a request of at least 94 bytes, all of them except for the booster fields are overridden by the [full resolution values](#full-resolution-values). Apps should still fill them in, so that an older package applies the closest values.

### Time Constants

Each byte is a `uint8` with the time constant in units of 10 ms. 0 is treated as 1.

| Offset | Field |
|--------|-------|
| 19     | `atr.filter.time_constant` |
| 20     | `atr.filter.on_speed_time_constant` |
| 21     | `atr.filter.off_speed_time_constant` |
| 22     | `torque_tilt.filter.time_constant` |
| 23     | `torque_tilt.filter.on_speed_time_constant` |
| 24     | `torque_tilt.filter.off_speed_time_constant` |
| 25     | `turn_tilt.filter.time_constant` |

### Full Resolution Values

Each value takes 2 bytes, big endian, and is clamped to its min/max. The scales and limits are the same as in `settings.xml`.

Encodings:
- `int16 ×S`: Signed 16-bit integer equal to the value multiplied by `S` (the same as `vTxDoubleScale` in `settings.xml`).
- `uint16`: Unsigned 16-bit integer.

There is no way to skip a value, all of them have to be present.

| Offset | Field                                | Description                        | Unit     | Encoding        | Min | Max   |
|--------|--------------------------------------|------------------------------------|----------|-----------------|-----|-------|
| 26     | `kp`                                 | Angle P                            |          | `int16 ×10`     | 0   | 60    |
| 28     | `kp2`                                | Rate P                             |          | `int16 ×100`    | 0   | 3     |
| 30     | `ki`                                 | Angle I                            |          | `int16 ×100000` | 0   | 0.3   |
| 32     | `ki_limit`                           | I Term Limit                       | A        | `int16 ×10`     | 0   | 500   |
| 34     | `mahony_kp`                          | Pitch KP                           |          | `int16 ×10000`  | 0.2 | 3     |
| 36     | `mahony_kp_roll`                     | Roll KP                            |          | `int16 ×10000`  | 0   | 3     |
| 38     | `kp_brake`                           | Angle P (Braking)                  | ×        | `int16 ×100`    | 0.2 | 3     |
| 40     | `kp2_brake`                          | Rate P (Braking)                   | ×        | `int16 ×100`    | 0   | 3     |
| 42     | `torquetilt_strength`                | Torque Tiltback: Strength          | °/A      | `int16 ×1000`   | 0   | 1     |
| 44     | `torquetilt_strength_regen`          | Torque Tiltback: Strength (Regen)  | °/A      | `int16 ×1000`   | 0   | 1     |
| 46     | `torquetilt_start_current`           | Start Current Threshold            | A        | `int16 ×100`    | 0   | 100   |
| 48     | `torquetilt_angle_limit`             | Torque Tiltback: Angle Limit       | °        | `int16 ×100`    | 0   | 30    |
| 50     | `torque_tilt.filter.on_speed_limit`  | Torque Tiltback: Max Speed         | °/s      | `int16 ×100`    | 0   | 100   |
| 52     | `torque_tilt.filter.off_speed_limit` | Torque Tiltback: Max Release Speed | °/s      | `int16 ×100`    | 0   | 100   |
| 54     | `turntilt_strength`                  | Turn Tiltback: Strength            |          | `int16 ×100`    | -30 | 30    |
| 56     | `turntilt_angle_limit`               | Turn Tiltback: Angle Limit         | °        | `int16 ×100`    | 0   | 30    |
| 58     | `turntilt_start_erpm`                | ERPM Threshold                     | ERPM     | `uint16`        | 100 | 10000 |
| 60     | `turntilt_erpm_boost`                | Speed Boost %                      | %        | `uint16`        | 0   | 10000 |
| 62     | `turntilt_erpm_boost_end`            | Speed Boost Max ERPM               | ERPM     | `uint16`        | 100 | 50000 |
| 64     | `turntilt_yaw_aggregate`             | Turn Aggregate Target              | °        | `uint16`        | 50  | 180   |
| 66     | `turntilt_start_angle`               | Turn Aggregate Threshold           | °        | `int16 ×100`    | 0   | 45    |
| 68     | `atr_strength_up`                    | ATR Uphill Strength                |          | `int16 ×1000`   | 0   | 3.5   |
| 70     | `atr_strength_down`                  | ATR Downhill Strength              |          | `int16 ×1000`   | 0   | 3.5   |
| 72     | `atr_threshold_up`                   | Threshold Angle Up                 | °        | `int16 ×100`    | 0   | 5     |
| 74     | `atr_threshold_down`                 | Threshold Angle Down               | °        | `int16 ×100`    | 0   | 5     |
| 76     | `atr_speed_boost`                    | ATR Speed Boost                    | fraction | `int16 ×10000`  | -1  | 1     |
| 78     | `atr_angle_limit`                    | ATR: Angle Limit                   | °        | `int16 ×100`    | 0   | 30    |
| 80     | `atr.filter.on_speed_limit`          | ATR: Max Tiltback Speed            | °/s      | `int16 ×100`    | 0   | 100   |
| 82     | `atr.filter.off_speed_limit`         | ATR: Max Tiltback Release Speed    | °/s      | `int16 ×100`    | 0   | 100   |
| 84     | `atr.transition_boost`               | Tiltback Transition Boost          | ×        | `int16 ×1000`   | 1   | 4     |
| 86     | `atr_amps_accel_ratio`               | Amps to Acceleration Ratio         |          | `int16 ×100`    | 5   | 30    |
| 88     | `atr_amps_decel_ratio`               | Amps to Deceleration Ratio         |          | `int16 ×100`    | 4   | 30    |
| 90     | `braketilt_strength`                 | Brake Tilt Strength                |          | `int16 ×100`    | 0   | 20    |
| 92     | `braketilt_lingering`                | Brake Tilt Lingering               |          | `int16 ×1000`   | 1   | 5     |

## Adding Fields

New fields are only ever appended to the end of the request, with a new minimum length. The existing part of the request must not change.
