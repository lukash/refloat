# Realtime Value Tracking

The package provides an easy-to-use mechanism to track realtime values which are not provided by default. It's as easy as modifying the list of values in [rt_data.h](/src/rt_data.h) and recompiling the package. The values are automatically propagated to the package UI (and 3rd party client apps, if they support it) via the [REALTIME_DATA](commands/REALTIME_DATA.md) command.

## Data Recording

The package allows to record a short period of realtime data into an internal memory buffer at the native loop frequency. This functionality requires a special firmware that reserves the buffer in memory and passes its address and size to the package. This firmware can be found here: https://github.com/lukash/bldc/tree/release_6_05-datarecord

The firmware with this information is automatically detected and advertised via `capabilities` in the [INFO](commands/INFO.md) command. In the package UI, the controls automatically appear in such case.

The values that are recorded are defined along with the realtime values in [rt_data.h](/src/rt_data.h) and can be easily added or removed. The length of the recorded period depends on the amount of values. More values, shorter period fits into the buffer.

On the command interface, the recording can be controlled via the [DATA_RECORD](commands/DATA_RECORD.md) command.
