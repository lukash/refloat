# Command: DATA_RECORD

This command is compound, it has one request message and two response messages.

It serves for controlling the data recording capability of the package. See [Realtime Value Tracking](../realtime_value_tracking.md) for more details.

## DATA_RECORD_REQUEST (Request)

**ID**: 41

A single request command to control the data recording functionality.

| Offset | Size | Name      | Mandatory | Description   |
|--------|------|-----------|-----------|---------------|
| 0      | 1    | `mode`    | Yes       | `1`: Control<br> `2`: Send<br> |
| 1      | 1    | `sub_mode`| Yes       | Submode is different for each of the two modes. |

### Mode: Control

Control mode expects one additional parameter:

| Offset | Size | Name    | Mandatory | Description   |
|--------|------|---------|-----------|---------------|
| 0      | 1    | `value` | Yes       | Represents a boolean value to set for given `sub_mode`. |

- **`sub_mode = 1`: Start/Stop Recording**
`value = 1` starts recording, `value = 0` stops recording.

- **`sub_mode = 2`: Set Autostart to `value`**.\
_Autostart will automatically start recording when engaging the board. Default value: True_

- **`sub_mode = 3`: Set Autostop to `value`**.\
_Autostop will automatically stop recording when disengaged. Default value: True_

### Mode: Send

Requests sending the data. Send mode has two submodes:

- **`sub_mode = 1`: Send Header**
- **`sub_mode = 2`: Send Data**

The Send Header submode should be sent first to get the metadata, followed by a series of Send Data requests.

#### Send Header

This submode has no additional parameters and the package will respond with the `DATA_RECORD_HEADER` Response.

_Note: Requesting the header will pause the recording. If the recording would continue while the data are being fetched, it would overwrite them as they are being read, there is no safeguard._

#### Send Data

This submode has one additional parameter:

| Offset | Size | Name     | Mandatory | Description   |
|--------|------|----------|-----------|---------------|
| 0      | 4    | `offset` | Yes       | Offset in number of samples to send. |

The client is responsible to request all the chunks of data by repeatedly calling this command with the offsets of the data it needs to fetch.

## DATA_RECORD_HEADER (Response)

**ID**: 42

Response with the metadata for the recorded data. Sends the total number of samples and then a list of [string](string.md) IDs for the values in each sample, same as in [REALTIME_DATA](REALTIME_DATA.md).

| Offset | Size | Name                     | Description   |
|--------|------|--------------------------|---------------|
| 0      | 4    | `size`                   | Size of the buffer in number of samples. The client is expected to fetch up to this number of samples. |
| 4      | 1    | `recorded_data_id_count` | Number of the recorded items per sample (sample size). |
| 5      | ?    | `recorded_data_ids`      | A [string](string.md) sequence repeated `recorded_data_id_count` times. |

## DATA_RECORD_DATA (Response)

**ID**: 43

Response with the actual sample data at the `offset` given in the request.

| Offset | Size | Name                     | Description   |
|--------|------|--------------------------|---------------|
| 0      | 4    | `offset`                 | `offset` repeated in the response. |
| 4      | ?    | `samples` | An unspecified number of `sample`s follows until the end of the message. |

**`sample`**:
| Offset | Size | Name     | Description   |
|--------|------|----------|---------------|
| 0      | 4    | `time`   | Timestamp of the sample in ticks, as `uint32`. |
| 4      | 1    | `flags`  | Important runtime flags and state. |
| 5      | ?    | `values` | A sequence of sample values, each 16 bits long and encoded in [float16](float16.md). The number of values is equal to the number of IDs sent in `DATA_RECORD_HEADER`. |

#### flags

|   7-4 |             3-2 |           1 |         0 |
|-------|-----------------|-------------|-----------|
| `sat` | `footpad_state` | `wheelslip` | `running` |

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
