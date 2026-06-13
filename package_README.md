# Refloat 1.3

A full-featured self-balancing skateboard package.

## New in 1.3
This version reworks a lot of the core control mechanisms and adds Setpoint Smoothing to all Tilts. Due to this, boards will behave slightly differently and tunes might need adjusting.

Changelog highlights:
- Reworked internal timing for better vibration rejection and more consistent tune behavior across setups
- Setpoint Smoothing for all Tilts (ATR, Torque Tilt, Brake Tilt, Turn Tilt, Remote)
- Torque normalization via motor Flux Linkage (firmware 6.06+)
- New and more consistent Reverse Stop
- Fullscreen remote overlay in package UI
- ...and many more features and fixes

For more details, read the [1.3 release post](https://pev.dev/t/refloat-version-1-3/2995).

## Support the project
I invest a lot of time and energy into the development and testing of Refloat. If you would like to support the development, here's [a few options to do so](https://riddimrider.one/donate/).

## Installation
### Upgrading
Back up your package config just in case (either by **Backup Configs** on the Start page, or by saving the XML in **Refloat Cfg**).

Unless upgrading from 1.0, an automatic config restore will pop up, confirm it. If this fails, restore the manual backup.

### Fresh Installation
If doing a fresh board installation, you need to do the **motor** and **IMU** calibration and configuration. If you install the package before that, you need to disable the package before running the **motor** _calibration_ and re-enable it afterwards.

For a detailed guide, read the [Initial Board Setup guide on pev.dev](https://pev.dev/t/initial-board-setup-in-vesc-tool/2190).

On (legacy) firmware 6.02, the **Low and High Tiltback voltages** in the **Specs** tab of **Refloat Cfg** need to be set according to your battery specs.

## Disclaimer
**Use at your own risk!** Electric vehicles are inherently dangerous, authors of this package shall not be liable for any damage or harm caused by errors in the software. Not endorsed by the VESC project.

## Credits
Author: Lukáš Hrázký

Original Float package authors: Mitch Lustig, Dado Mista, Nico Aleman

## Downloads and Changelogs
[https://github.com/lukash/refloat/releases](https://github.com/lukash/refloat/releases)
