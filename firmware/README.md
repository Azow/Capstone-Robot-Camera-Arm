# Arm firmware (Octopus Pro v1.1, STM32H723)

Firmware for the BigTreeTech Octopus Pro v1.1 that drives the arm's stepper motors.
Design decisions are in [`../docs/control_system_decisions.md`](../docs/control_system_decisions.md).

**Status: bring-up.** A text command shell for testing the board by hand: check the drivers
answer, enable joints, spin a motor, read the endstops. The real protocol and the timed
step generator come next (build-order steps 4+ in the decisions doc).

## Files

| File | What it is |
|---|---|
| `platformio.ini` | Build setup: Nucleo-H723ZG base, 128 KiB bootloader offset, 25 MHz crystal, USB serial |
| `src/board_pins.h/.cpp` | Octopus Pro v1.1 pin map; turns every driver, heater and fan off at boot |
| `src/clock_config.cpp` | Clock setup for the 25 MHz crystal (400 MHz CPU) |
| `src/config.h` | Which joint is on which slot, driver type, current, microsteps |
| `src/drivers.h/.cpp` | TMC5160 (SPI) and TMC2209 (UART) setup, enable pins |
| `src/shell.h/.cpp` | Bring-up command shell over USB serial |
| `src/main.cpp` | Startup and main loop |

## Slot assignment (D-16)

| Slot | Joint | Driver | Supply | Mode jumpers | Endstop |
|---|---|---|---|---|---|
| Driver0 | Theta | TMC5160T Pro | 48 V | SPI | STOP_0 |
| Driver1 | Z1 | TMC5160T Pro | 48 V | SPI | STOP_1 (the Z switch) |
| Driver2 | Z2 | TMC5160T Pro | 48 V | SPI | none |
| Driver3 | R | TMC5160T Pro | 48 V | SPI | STOP_3 |
| Driver4 | Yaw | TMC2209 | **24 V** | UART | STOP_4 |
| Driver5 | Pitch | TMC2209 | **24 V** | UART | STOP_5 |
| Driver6, 7 | spare | | | | STOP_2, 6, 7 spare (e-stop later) |

## Before the first power-on

Check all of these against the BTT Octopus Pro v1.1 manual.

- [ ] **Driver4 and Driver5 (TMC2209) are powered from 24 V, NOT the 48 V motor input.**
      A TMC2209 on 48 V is destroyed.
- [ ] Driver0–3 (TMC5160T Pro) are jumpered to the 48 V motor-power input, and the
      TMC5160T Pro is rated for 48 V with margin.
- [ ] Each slot's mode jumpers match its driver: SPI for Driver0–3, UART for Driver4–5.
- [ ] DIAG jumpers removed on the slots whose endstop inputs we use.
- [ ] Driver orientation in the sockets (EN pin to EN): a reversed driver is destroyed.
- [ ] `kRSense5160` in `src/config.h` matches the TMC5160T Pro's sense resistor
      (if the code's value is higher than the real one, the motors get more current than asked).
- [ ] The Z carriage is supported: Z falls whenever its drivers are disabled (D-31).

## Build

Install PlatformIO: the VS Code extension, or `pip install platformio` in a virtual env.

```bash
cd ~/capstone_ws/firmware
pio run
```

Output: `.pio/build/octopus_pro_h723/firmware.bin`.

## Flash

**SD card (default, keeps the stock bootloader):**
1. Copy `firmware.bin` to the root of a FAT32 microSD card, named exactly `firmware.bin`.
2. Power off the board, insert the card, power on. Wait ~10 s.
3. The bootloader flashes it and renames the file to `FIRMWARE.CUR`: that means it worked.

**USB DFU (alternative):** fit the BOOT0 jumper, press reset, then `pio run -t upload`.
Remove the jumper and reset afterwards. Writes at the same offset, so the bootloader stays.

## Connect from WSL

The board appears on Windows as a USB serial port. Attach it to WSL with
[usbipd-win](https://github.com/dorssel/usbipd-win), from an admin PowerShell:

```powershell
usbipd list                      # find the board's BUSID
usbipd bind --busid <BUSID>      # once
usbipd attach --wsl --busid <BUSID>   # every time it's plugged in
```

Then in WSL it shows up as `/dev/ttyACM0`:

```bash
pio device monitor --echo -p /dev/ttyACM0
```

## Commands

One per line, not case-sensitive. Every reply ends in a line starting `OK` or `ERR`.

| Command | What it does |
|---|---|
| `HELP` | List commands |
| `PING` | Replies `OK` |
| `INFO` | Firmware version and CPU clock (should be 400000000 Hz) |
| `DRIVERS` | Configures every driver and checks it answers. Needs motor power. Run again after motor power is cycled (drivers forget their settings). |
| `ENABLE <joint\|ALL>` | Energize a joint (Z = both Z drivers) |
| `DISABLE <joint\|ALL>` | De-energize. **Z falls unless supported.** |
| `SPIN <joint> <steps> [rate]` | Move a joint at a constant rate (steps/s, default 400, no acceleration). Negative steps reverse. Type anything to abort. |
| `ENDSTOPS` | Raw endstop levels (pulled up: 1 = high, also what a disconnected sensor reads) |

Joints: `THETA Z R YAW PITCH`.

## First test, in order

1. USB only, no motor power: `PING`, then `INFO` (checks the clock setup).
2. `ENDSTOPS` while blocking/unblocking each optical gate by hand: note which level means
   blocked, for D-24.
3. Motor power on: `DRIVERS`. Every driver should report its version (TMC5160 `0x30`,
   TMC2209 `0x21`). `NO ANSWER` = motor power off, wrong mode jumpers, or wrong slot.
4. One motor, joint free to move or disconnected from the mechanism:
   `ENABLE YAW`, then `SPIN YAW 3200 400` (one turn at 16 microsteps, 200-step motor).
5. Z only with the carriage supported: `ENABLE Z`, `SPIN Z 400 200`. Check both motors turn
   the same way; if not, set `invert_dir` on Z2 in `src/config.h`.

The currents in `src/config.h` are low placeholders (400 mA / 300 mA) until the real motors
are known (D-14).
