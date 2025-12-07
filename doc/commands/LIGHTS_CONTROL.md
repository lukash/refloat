# Command: LIGHTS_CONTROL

**ID**: 20

Controls the lights and returns current lighting state values.

## Request

| Offset | Size | Name      | Mandatory | Description   |
|--------|------|-----------|-----------|---------------|
| 0      | 4    | `mask`    | No        | Mask determining which values are being set in the command. Bits 0..7 correspond to the `flags` bits. Bits 8..31 are unused. |
| 4      | 1    | `flags`   | No        | Values of lighting flags to set. |

## Response

| Offset | Size | Name    | Description   |
|--------|------|---------|---------------|
| 0      | 1    | `flags` | Current values of lighting flags. |

## Common Structures

#### flags

| 7-2 |               1 |         0 |
|-----|-----------------|-----------|
|   0 | `headlights_on` | `leds_on` |
