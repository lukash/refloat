# Refloat

A full-featured self-balancing skateboard package.

## New in 1.2
- BMS alerting support (Pushback and Haptic)
- Speed-based alerting (Pushback and Haptic)
- Automatic config migration when updating the package
- Support for per-cell High/Low Voltage Thresholds (no more needing to adjust them in Specs after a fresh install, 6.05+ only)
- Groundworks for a new alerting system, for now only providing firmware fault reporting
- New Rainbow Fade, Rainbow Cycle and Rainbow Roll LED effects

For more details, read the [1.2 release post](https://pev.dev/t/refloat-version-1-2/2795).

## Installation
### Upgrading
Back up your package config (either by **Backup Configs** on the Start page, or by saving the XML on **Refloat Cfg**).

If upgrading from Refloat 1.1 to 1.2, an automatic config restore dialog should pop up. Confirm it to restore. This is the preferred way to update the config from now on, as it will also migrate any changed options to the new version. Other methods of restoring the config will not do that.

In case the automatic config restore fails, restore it the traditional way and please report the issue, so that it can be fixed.

### Fresh Installation
If doing a fresh board installation, you need to do the **motor** and **IMU** calibration and configuration. If you install the package before that, you need to disable the package before running the **motor** _calibration_ and re-enable it afterwards.

For a detailed guide, read the [Initial Board Setup guide on pev.dev](https://pev.dev/t/initial-board-setup-in-vesc-tool/2190).

On 6.05+ firmware, the package should ride well without the need to configure anything in Refloat Cfg. On 6.02, the **Low and High Tiltback voltages** on the **Specs** tab of **Refloat Cfg** still need to be set according to your battery specs.

## Disclaimer
**Use at your own risk!** Electric vehicles are inherently dangerous, authors of this package shall not be liable for any damage or harm caused by errors in the software. Not endorsed by the VESC project.

## Credits
Author: Lukáš Hrázký

Original Float package authors: Mitch Lustig, Dado Mista, Nico Aleman

## Downloads and Changelogs
[https://github.com/lukash/refloat/releases](https://github.com/lukash/refloat/releases)
