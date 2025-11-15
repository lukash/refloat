# Debugging Refloat

The package can be debugged in GDB the standard way if the package symbols are loaded from the package .elf file.

## 1. Start the OpenOCD server

Start the OpenOCD server using the configuration file from the repository:
```sh
$ openocd -f openocd.cfg
```

Keep this process running in its own terminal.

## 2. Connect to OpenOCD server with GDB

An ARM version of GDB is needed, and it needs to have a working Python scripting support.

In a second terminal, change to the `refloat` repo directory. It contains the `.gdbinit` file with commands to load the package .elf and for debuging it.

The correct firmware .elf file is needed, it's built along with the firmware binary for the ESC in question:
```sh
$ arm-none-eabi-gdb path/to/bldc/build/ESC/ESC.elf -ex 'target extended-remote localhost:3333'
```

## 3. Load the package .elf

Once inside GDB, load the symbols for the package itself:
```gdb
(gdb) load-package-elf src/package_lib.elf
```

## 4. Debug the package

You can now use package symbols, set breakpoints, decode package frames in stack traces etc. The package has a global `data` struct, managed and made available through the firmware. The `.gdbinit` file provides a helper command to access this data easily at any point:
- Type `data` to print the contents of the whole struct
- Type `data PROPERTY` to print any property of the data struct, even nested, e.g.:
  ```gdb
  (gdb) data imu
  $1 = {
    pitch = 21.3763847,
    balance_pitch = 21.2723846,
    roll = -1.01360452,
    yaw = 32.8550987,
    pitch_rate = 0.0440952145,
    flywheel_pitch_offset = 0,
    flywheel_roll_offset = 0
  }
  (gdb) data imu.pitch
  $2 = 21.3763847
  ```
_Note: A recent enough GCC version is needed for compiling the package, otherwise reading the `Data` struct in GDB won't work (it doesn't work with GCC 7)._
