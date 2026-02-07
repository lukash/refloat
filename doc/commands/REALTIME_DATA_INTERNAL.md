# Command: REALTIME_DATA_INTERNAL

**ID**: 31

Provides realtime data from the package to the client.

This command is internal to the package and compatibility of its interface is not guaranteed. Using this command in 3rd party clients is not recommended.

## Request

No data.

## Response

The beginning of the response contains information about the state of the package. Following it is a sequence of realtime values in the 2-byte [float16](float16.md) format. Depending on the `mask`, these are optionally followed by a sequence of realtime _runtime_ values or charging values.

The definition of what realtime and realtime _runtime_ values are sent is provided in the [REALTIME_DATA_IDS](REALTIME_DATA_IDS.md) command as two sequences of string IDs.

The actual list of values sent is in [rt_data.h](/src/rt_data.h). See also [Realtime Value Tracking](../realtime_value_tracking.md) for more details.

| Offset | Size | Name                  | Description   |
|--------|------|-----------------------|---------------|
| 0      | 1    | `mask`                | Mask which specifies which data are included in the response:<br> `0x1`: Runtime data<br> `0x2`: Charging data |
| 1      | 1    | `extra_flags`         | Special internal package flags. |
| 2      | 4    | `time`                | Timestamp of the data in ticks, as `uint32`. To convert to seconds, use `tick_rate` from the [INFO](INFO.md) command. |
| 6      | 4    | `state_flags`         | State flags for the internal package state. |
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

See [extra_flags](REALTIME_DATA.md#extra_flags) in the REALTIME_DATA command.

#### state_flags

See [state_flags](REALTIME_DATA.md#state_flags) in the REALTIME_DATA command.
