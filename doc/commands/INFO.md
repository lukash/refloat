# Command: INFO

**ID**: 0

Serves for initiating communication with the package and provides information about the package.

This command is versioned, the client requests a version of this command and the package returns the response of that version if it supports it. If the version is higher than what the package supports, it responds with the highest version it knows.

If the client would request a version that is lower than what the package supports, the package should respond with a version `0` in its response to indicate it no longer supports a version this low.

_Note: What is now referred to as version `1` of this command originally had no notion of versioning,but the mechanism is designed so that the version `1` is compatible with the original command._

## Request

| Offset | Size | Name      | Mandatory | Description   |
|--------|------|-----------|-----------|---------------|
| 0      | 1    | `version` | No        | Requested version of the INFO command. In case the package doesn't support version this high, it shall respond with the highest version of the command it supports. Default value: `1` |
| 1      | 1    | `flags`   | No        | Only for version 2. Flags for toggling extra information in the response:<br> `0x1`: Include `REALTIME_DATA` IDs in the response. See the [REALTIME_DATA](REALTIME_DATA.md) command for more information. |

## Response

### version 1

| Offset | Size | Name                  | Description   |
|--------|------|-----------------------|---------------|
| 0      | 1    | `major_minor_version` | Package major and minor version encoded in a single number: `major * 10 + minor` |
| 1      | 1    | `build_number`        | Always `1`.                          |
| 2      | 1    | `leds_type`           | LEDs type:<br> `0`: None<br> `1`: Internal<br> `3`: External <br> _Note: In this version, "External" is `3` and `2` is never returned._ |

### version 2

| Offset | Size | Name                     | Description   |
|--------|------|--------------------------|---------------|
| 0      | 1    | `version`                | The actual version of this `INFO` command response, for the case a lower than requested version is returned.<br> _Note: version 1 does not return this, but its first byte in the response is always higher than 10, which can be used to distinguish the version._ |
| 1      | 1    | `flags`                  | The `flags` from the request repeated in the response, to indicate which extra data are present. |
| 2      | 1    | `package_major_version`  | Major version of the package. |
| 3      | 1    | `package_minor_version`  | Minor version of the package. |
| 4      | 1    | `package_patch_version`  | Patch version of the package. |
| 5      | 16   | `package_version_suffix` | An optional package version suffix. Zero-padded to 16 bytes, but not zero-terminated in case all 16 bytes are used. |
| 21     | 4    | `tick_rate`              | Tick rate of the system in Hz. This number can be used to convert time measured in ticks in other commands (namely `REALTIME_DATA`) to seconds by dividing by this number. Currently the tick rate for VESC is always `10000`. |
| 25     | 4    | `capabilities`           | Capability flags of the package:<br> `0x80000000`: Data Recording. See the [Realtime Value Tracking](../realtime_value_tracking.md) page for details. |
| 29     | 1    | `leds_mode`              | LEDs mode:<br> `0`: None<br> `1`: Internal<br> `2`: External |

#### flags

Flags indicate extra data in the response. The data follow in the response in the order of the flags as they are listed (for each flag that is enabled).

**`0x1`: [REALTIME_DATA](REALTIME_DATA.md) command IDs**

Contains two sequences of string IDs for data that are sent in the [REALTIME_DATA](REALTIME_DATA.md) command.

The actual list of IDs that are sent is in [rt_data.h](/src/rt_data.h). See [Realtime Value Tracking](../realtime_value_tracking.md) for more details.

| Offset | Size | Name                             | Description   |
|--------|------|----------------------------------|---------------|
| 0      | 1    | `realtime_data_id_count`         | Number of IDs of the `REALTIME_DATA` items which are always sent. |
| 1      | ?    | `realtime_data_ids`              | A `string` sequence repeated `realtime_data_id_count` times. |
| ?      | 1    | `realtime_runtime_data_id_count` | Number of IDs of the `REALTIME_DATA` items which are sent only when the package is running. |
| ?      | ?    | `realtime_runtime_data_ids`      | A `string` sequence repeated `realtime_runtime_data_id_count` times. |

**`string`**:
| Offset | Size | Name     | Description   |
|--------|------|----------|---------------|
| 0      | 1    | `length` | Length of the string. |
| 1      | ?    | `string` | `length` number of characters of the string (not null-terminated). |
