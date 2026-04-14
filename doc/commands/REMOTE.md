# Command: REMOTE

**ID**: 15

Controls the board remotely by providing a normalized input value. The input is used both for tilting the nose (remote tilt) and for driving the wheel (remote move). This command takes priority over PPM/UART remote input for 0.5 seconds after each message.

This is a one-way command with no response.

## Request

| Offset | Size | Name    | Mandatory | Description   |
|--------|------|---------|-----------|---------------|
| 0      | 1    | `value` | Yes       | Signed 8-bit integer in the range -127..127. Normalized internally to -1.0..1.0. The value `-128` is ignored. |
