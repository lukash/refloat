# Commands Reference

Commands are binary messages that can be used to interact with Refloat. They can be sent from a client device (e.g. a Bluetooth-connected phone app) or from a device component connected to the VESC bus (e.g. a VESC Express, or an UART device). All messages are broadcast to all components connected to the bus on VESC level (meaning above the package level). Each component decides whether it will process a given message.

On VESC level, there is a (VESC-level) command designated for package commands (it's called `COMM_CUSTOM_APP_DATA`). The payload of this command is what is sent and received by the package. For clients using a VESC API (e.g. a QML VESC Tool app) there's a function to send this command. Clients communicating with VESC outside of its ecosystem need to construct the message with its first byte set to this VESC command ID (the value of which is `36`). The rest of this documentation only deals with the payload of this VESC command and disregards the first byte.

The convention for package commands is the first byte of the message is the identifier of the package interface: `package_interface_id` (also formerly called `magic` in Float). Refloat currently only processes commands with `package_interface_id` = `101` (same as Float).

The second byte is the `command_id`. So each package command starts with what we could call a "command header" of the following format:

| Offset | Size | Name                   | Description                          |
|--------|------|------------------------|--------------------------------------|
| 0      | 1    | `package_interface_id` | Identifier of the package interface. |
| 1      | 1    | `command_id`           | Command ID.                          |

Messages can be freely sent both ways (from the package to anything that is listening, or from client components to the package), but typically they work on a request-response basis from client components towards the package. Some commands are one-way and don't have a response. In case of request-response, the aforementioned first two bytes are the same in the request and the response.

The request and response are not linked in any way besides the first two bytes. A consequence of this together with the broadcast mechanism is that if there are two components (clients) both interacting with a third one (say, a package), both of the clients will receive responses to requests of the other client and they have no easy way to tell them apart. This can lead to some nasty issues once there are more than two components communicating via the same commands.

All commands on the VESC bus are checksummed (so they shouldn't get mangled). In case of a checksum mismatch or any other communication issue, the message is dropped.

## Commands

In the commands' documentation, the first two bytes with `package_interface_id` and `command_id` are omitted, so while their offsets start at 0, in the full message their data are always preceded by them.

- [INFO](INFO.md)
- [LIGHTS_CONTROL](LIGHTS_CONTROL.md)
- [REALTIME_DATA](REALTIME_DATA.md)
- [REALTIME_DATA_IDS](REALTIME_DATA_IDS.md)
- [DATA_RECORD](DATA_RECORD.md)
- [ALERTS_LIST](ALERTS_LIST.md)
- [ALERTS_CONTROL](ALERTS_CONTROL.md)
