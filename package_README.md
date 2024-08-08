# Refloat
Self-balancing skateboard package based on the Float package by Mitch Lustig, Dado Mista and Nico Aleman: Float, Refactored.

The feature set of Refloat is the same as that of Float version 1.3, with the following features ported from Float 2.0:
- Floatwheel LCM support
- Negative ATR speed boost

Refloat 1.0 adds the following main features:
- **Separate Axis KP**
- **Brand new full-featured GUI in VESC Tool**
- **Advanced LED lighting control for LEDs connected directly to VESC**

Refloat configuration and tunes are 100% compatible with Float 1.3 (and hence also 2.0, apart from missing a few newly added options). Refloat also maintains compatibility with 3rd party apps (Floaty / Float Control), though it works best with its brand new VESC Tool interface.

#### Separate Axis KP
The Mahony KP can now be configured separately for each IMU axis. This feature improves the way the board handles in turns and in a significant way improves the balance profile.

Technical explanation: High Mahony KP works very well for balancing, but it is only desirable on the pitch axis. The mellow response of high KP has unwanted effects on the roll axis, because when the board rotates in yaw (when turning), roll becomes pitch. The board leads into a turn angled in roll (more pronounced on rounder tires), and this angle translates into pitch and lingers there for an amount of time determined by the KP, causing the nose to be down for a time, until it balances back up.

Lower roll KP makes the nose hold up better in turns. It makes the board more stable and "stiffer", especially in short carves. This means **the Turn Tilt feature may be less needed** and it may respond more aggressively.

Mahony configuration in Firmware App Config -> IMU is now a setting independent from Refloat. It is used for general purpose pitch, roll and yaw only (this pitch is called "true pitch" in Float). It is recommended to use Mahony KP 0.4 and Mahony KI 0 there, and these values will be set automatically by Refloat if Mahony KP > 1 is encountered in the App Config -> IMU configuration.

The Mahony KP setting has been renamed to Pitch KP (its effect is the same as of Mahony KP) and there is a new setting called Roll KP. These are now independent from Firmware App Config -> IMU.

Read the descriptions of the new configuration options for more information.

### Fresh Installation
It's best to configure your **motor** and **IMU** before installing the package. If you install the package first, a welcome dialog will navigate you to how to disable the package temporarily to configure the **motor**. You don't need to disable the package to configure the **IMU**, just make sure you don't activate the board before you do so, as it may and most likely will behave erratically.

After the **motor** and **IMU** setup, install (or re-enable) the package and configure the **Low and High Tiltback voltages** on the **Specs** tab of **Refloat Cfg**. These need to be set according to your battery specs.

The options in the other tabs are set to good starting values that should provide you with a well behaving, rideable board. If it acts weird, it's most likely a **motor** configuration or **IMU** calibration issue.

### Migrating from Float
Backup your configuration by exporting the XML, install Refloat and load the backup. All options known to Refloat will be loaded, which entails all Float 1.3 options (with the small exception of the **Specs -> Disable** toggle, which will default to "Enabled").

#### Tune Quicksaves
Tune quicksaves will be converted from Float to Refloat on first start. Your Float tunes won't be touched and if you go back to Float, you will find them unchanged.

IMU quicksaves were removed from Refloat.

#### Tune Archive
Refloat uses the same Tune archive as Float, the same tunes are available.

### Disclaimer
Use at your own risk. Electric vehicles are inherently dangerous, authors of this package shall not be liable for any damage or harm caused by errors in the software. Not endorsed by the VESC project.

### Credits
Author: Lukáš Hrázký

Original Float package authors: Mitch Lustig, Dado Mista, Nico Aleman

### Download and Changelog
[https://github.com/lukash/refloat/releases](https://github.com/lukash/refloat/releases)
