# Command: ALERTS_LIST

**ID**: 35

**Status**: **unstable**

Returns a list of active alerts and a log of alert history. The package maintains a number of last occurred alerts in a circular buffer. This command can return all of them, or, optionally provide a starting timestamp in ticks in the `since` argument. In such case only alerts which occurred after the timestamp are returned (note: the caller needs to take into account the possibility of the ticks overflowing, which occurs once every ~5 days).

The response is limited to 511B in size, in case the alerts wouldn't fit in the message, the alert list will be trucncated (hence it's recommended to use the `since` argument to deal with this eventuality).

## Request

| Offset | Size | Name    | Mandatory | Description   |
|--------|------|---------|-----------|---------------|
| 0      | 4    | `since` | No        | Timestamp in ticks from which to list alerts in the log as `uint32`. |

## Response

| Offset | Size | Name                  | Description   |
|--------|------|-----------------------|---------------|
| 0      | 4    | `active_alert_mask_1` | Bits 0..31 of the `active_alert_mask`, indicating which [alert_id](alert_id.md)s are active. |
| 4      | 8    | `active_alert_mask_2` | Bits 32..63 of the `active_alert_mask`, indicating which [alert_id](alert_id.md)s are active. |
| 8      | 1    | `firmware_fault_code` | In case `ALERT_FW_FAULT` is active, the VESC firmware fault code, otherwise 0. |
| 9      | ?    | `firmware_fault_name` | Iff `firmware_fault_code` is non-zero, a [string](string.md) containing the firmware fault name.
| ?      | ?    | `alert_log`           | A sequence of `alert_record`s containing the alert history stored in the package. |

**`alert_record`**:
| Offset | Size | Name     | Description   |
|--------|------|----------|---------------|
| 0      | 4    | `time`   | Timestamp of the alert in ticks, as `uint32`. |
| 4      | 1    | `id`     | The ID of the alert, see [alert_id](alert_id.md). |
| 5      | 1    | `active` | `1` if the event represents the alert becoming active, `0` if the event marks the end of the alert. |
| 6      | 1    | `code`   | If the Alert ID is `ALERT_FW_FAULT`, the code of the fault from the firmware, otherwise 0. |
| 7      | ?    | `firmware_fault_name` | Iff Alert ID is `ALERT_FW_FAULT` and `firmware_fault_code` is non-zero, a [string](string.md) containing the firmware fault name.
