# Command: REALTIME_DATA

**ID**: 31

Provides realtime data from the package to the client.

## Request

No data.

## Response

The beginning of the response contains information about the state of the package. Following it is a sequence of realtime values in the 2-byte [float16](float16.md) format. Depending on the `mask`, these are optionally followed by a sequence of realtime _runtime_ values or charging values.

The definition of what realtime and realtime _runtime_ values are sent is provided in the [REALTIME_DATA_IDS](REALTIME_DATA_IDS.md) command as two sequences of string IDs.

The actual list of values sent is in [rt_data.h](/src/rt_data.h). See also [Realtime Value Tracking](../realtime_value_tracking.md) for more details.

| Offset | Size | Name                  | Description   |
|--------|------|-----------------------|---------------|
| 0      | 1    | `mask`                | Mask which specifies which data are included in the response:<br> `0x1`: Runtime data<br> `0x2`: Charging data |
| 1      | 1    | `extra_flags`         | Extra flags for various internal package state values. |
| 2      | 4    | `time`                | Timestamp of the data in ticks, as `uint32`. To convert to seconds, use `tick_rate` from the [INFO](INFO.md) command. |
| 6      | 1    | `state_and_mode`      | The state and mode of the package. |
| 7      | 1    | `flags_and_footpad`   | The state flags and footpad state. |
| 8      | 1    | `stop_cond_and_sat`   | The stop condition and SAT (Setpoint Adjustment Type). |
| 9      | 1    | `alert_reason`        | The last alert reason. |
| 10     | N    | `realtime_data`       | The realtime data as a sequence of [float16](float16.md)-encoded numbers. |

#### Mask

In case the following bits are set in the `mask`, the listed data follow in the response in the order they are listed in.

**`0x1`: Runtime data**
| Offset | Size | Name                    | Description   |
|--------|------|-------------------------|---------------|
| 0      | N    | `realtime_runtime_data` | The realtime _runtime_ data as a sequence of [float16](float16.md)-encoded numbers. |

**`0x2`: Charging data**
| Offset | Size | Name                    | Description   |
|--------|------|-------------------------|---------------|
| 0      | 2    | `charging_current` | The charging current encoded as [float16](float16.md). |
| 2      | 2    | `charging_voltage` | The charging voltage encoded as [float16](float16.md). |

**`0x4`: Alerts**
| Offset | Size | Name                  | Description   |
|--------|------|-----------------------|---------------|
| 0      | 4    | `active_alert_mask_1` | Bits 0..31 of the `active_alert_mask`, indicating which [alert_id](alert_id.md)s are active. |
| 4      | 8    | `active_alert_mask_2` | Bits 32..63 of the `active_alert_mask`, indicating which [alert_id](alert_id.md)s are active. |
| 8      | 1    | `firmware_fault_code` | In case `ALERT_FW_FAULT` is active, the VESC firmware fault code, otherwise 0. |

#### extra_flags

| 7-3 |                      2 |                       1 |                       0 |
|-----|------------------------|-------------------------|-------------------------|
|   0 | `data_record_autostop` | `data_record_autostart` | `data_record_recording` |

The `data_record_*` flags represent data recording internal state, see [DATA_RECORD](DATA_RECORD.md).

#### state_and_mode

| 7-6 |            5-4 | 3-2 |             1-0 |
|-----|----------------|-----|-----------------|
|   0 | `package_mode` |   0 | `package_state` |

**`package_state`**:
- `0: DISABLED`
- `1: STARTUP`
- `2: READY`
- `3: RUNNING`

**`package_mode`**:
- `0: NORMAL`
- `1: HANDTEST`
- `2: FLYWHEEL`

#### flags_and_footpad

|             7-6 |          5 | 3-2 |          1 |           0 |
|-----------------|------------|-----|------------|-------------|
| `footpad_state` | `charging` |   0 | `darkride` | `wheelslip` |

**`footpad_state`**:
- `0: NONE`
- `1: LEFT`
- `2: RIGHT`
- `3: BOTH`

#### stop_cond_and_sat

|   7-4 |              3-0 |
|-------|------------------|
| `sat` | `stop_condition` |

**`stop_condition`**:
- `0: NONE`
- `1: PITCH`
- `2: ROLL`
- `3: SWITCH_HALF`
- `4: SWITCH_FULL`
- `5: REVERSE_STOP`
- `6: QUICKSTOP`

**`sat` (setpoint adjustment type)**:
- `0: NONE`
- `1: CENTERING`
- `2: REVERSESTOP`
- `6: PB_DUTY`
- `10: PB_HIGH_VOLTAGE`
- `11: PB_LOW_VOLTAGE`
- `12: PB_TEMPERATURE`

#### alert_reason

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
