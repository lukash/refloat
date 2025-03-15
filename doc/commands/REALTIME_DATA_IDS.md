# Command: REALTIME_DATA_IDS

**ID**: 32

Returns a list of string IDs of the items sent in the [REALTIME_DATA](REALTIME_DATA.md) command. The number of items in that command, as well as their meaning, is defined by the two sequences of IDs obtained from this command.

## Request

No data.

## Response

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
