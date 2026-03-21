# Refloat 1.3 feature preview 1

A full-featured self-balancing skateboard package.

This is a feature preview of the upcoming 1.3 version, meant to showcase new features and gather feedback.

## New in 1.3
This version reworks a lot of the core control mechanisms and corrects a number of minor inaccuracies, which impacts behavior. Due to this, as well as due to the added Setpoint Smoothing, boards will behave slightly differently and tunes might need adjusting.

Changelog highlights:
- Improved package-side control loop timing
- Independence of config options and internal constants on the loop frequency
- Compensation for different motors' torque constant (6.06+)
- Setpoint smoothing for all tilts (Torque Tilt, ATR, Turn Tilt, Brake Tilt, Remote)

For more details, read the [1.3-preview1 release post](https://pev.dev/t/relfoat-1-3-feature-preview-1-timing-rework-and-setpoint-smoothing/2959).

## Installation
### Upgrading
Back up your package config just in case (either by **Backup Configs** on the Start page, or by saving the XML on **Refloat Cfg**).

Unless upgrading from 1.0, an automatic config restore will pop up, confirm it. If this fails, restore the manual backup.

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
