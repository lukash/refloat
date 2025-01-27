# Realtime Value Tracking

The package provides an easy-to-use mechanism to track realtime values which are not provided by default. It's as easy as modifying the list of values in [rt_data.h](/src/rt_data.h) and recompiling the package. The values are automatically propagated to the package UI (and 3rd party client apps, if they support it) via the [REALTIME_DATA](commands/REALTIME_DATA.md) command.
