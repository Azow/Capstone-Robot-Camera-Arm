# Arm firmware (Octopus, STM32F446ZE)

Firmware for the BigTreeTech Octopus with the STM32F446ZE (Octopus V1.1 / Pro V1.0 pin set)
that drives the arm's stepper motors.
Design decisions are in [`../docs/control_system_decisions.md`](../docs/control_system_decisions.md).

**Status: protocol v1 and the step generator (build step 4), untested on hardware.** The
board speaks the serial protocol (decisions doc, section 4) that `arm_bridge` uses, queues
timed segments, and steps every joint from a 50 kHz timer interrupt. The bring-up commands
(`INFO`, `DRIVERS`, `ENDSTOPS`, `SPIN`) are still there for testing by hand. Homing (step 5)
is not written yet: `HOME` answers `NOT_IMPLEMENTED`; use `SETPOS` on the bench.

The protocol and motion code (`lib/arm_core/`) has no Arduino code in it, so it is also
built for the PC: unit tests, and a simulator that `arm_bridge` can drive.

## Files

| File | What it is |
|---|---|
| `platformio.ini` | Build setup: board build, PC unit tests, simulator |
| `boards/octopus_f446.json` | PlatformIO board: STM32duino's generic F446ZE variant |
| `src/board_pins.h/.cpp` | Octopus F446 pin map; turns every driver, heater and fan off at boot |
| `src/clock_config.cpp` | Clock setup for the 12 MHz crystal (180 MHz CPU, USB from PLLSAI) |
| `src/config.h` | Which joint is on which slot, driver type, current, microsteps |
| `src/drivers.h/.cpp` | TMC5160 (SPI) and TMC2209 (UART) setup, enable pins |
| `src/octopus_hal.h/.cpp` | Fast STEP/DIR pin access for the step interrupt; bring-up commands |
| `src/main.cpp` | Startup, the 50 kHz step timer, USB serial lines to the protocol |
| `lib/arm_core/src/motion.h/.cpp` | Segment queue and step generator (Bresenham), ramp-down on stop |
| `lib/arm_core/src/protocol.h/.cpp` | The serial protocol: commands, checks, replies, events, watchdog |
| `lib/arm_core/src/hal.h` | What `arm_core` needs from the hardware |
| `src/sim/sim_main.cpp` | PC simulator: `arm_core` on a pseudo-terminal (not in the board build) |
| `test/test_core/` | Unit tests for `arm_core`, run on the PC |
| `tools/make_arduino_sketch.py` | Copies `src/` and `lib/arm_core/` into the Arduino sketch |
| `arduino/arm_firmware/` | **Generated** Arduino IDE sketch: what gets flashed. Don't edit it. |

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

Check all of these against the BTT Octopus manual for this board.

- [ ] **Driver4 and Driver5 (TMC2209) are powered from 24 V, NOT the 48 V motor input.**
      A TMC2209 on 48 V is destroyed.
- [ ] This board takes 48 V motor power at all (the Octopus Pro has a 60 V `MOTOR_POWER`
      input; the plain Octopus V1.1 is limited to about 28 V). See D-34.
- [ ] Driver0–3 (TMC5160T Pro) are jumpered to the 48 V motor-power input, and the
      TMC5160T Pro is rated for 48 V with margin.
- [ ] Each slot's mode jumpers match its driver: SPI for Driver0–3, UART for Driver4–5.
- [ ] DIAG jumpers removed on the slots whose endstop inputs we use.
- [ ] Driver orientation in the sockets (EN pin to EN): a reversed driver is destroyed.
- [ ] `kRSense5160` in `src/config.h` (0.075 Ω, from BTT's wiki) matches the sense resistors
      on the TMC5160T Pro (marked R075; an R022 board needs 0.022). If the code's value is
      higher than the real one, the motors get more current than asked: 3.4x for R022.
- [ ] The Z carriage is supported: Z falls whenever its drivers are disabled (D-31).

## Build

The firmware is edited here, as a PlatformIO project, and flashed from Arduino IDE.

PlatformIO (VS Code extension, or `pip install platformio` in a virtual env):

```bash
cd ~/capstone_ws/firmware
pio run                     # board build, catches compile errors
```

After changing anything in `src/` or `lib/arm_core/`, regenerate the Arduino sketch and commit
it with the change:

```bash
python3 tools/make_arduino_sketch.py           # writes arduino/arm_firmware/
python3 tools/make_arduino_sketch.py --check   # fails if it's out of date
```

## Test on the PC (no board needed)

```bash
pio test -e native          # unit tests for lib/arm_core
pio run -e sim              # the simulator
.pio/build/sim/program --link /tmp/sim_octopus
```

`picocom /tmp/sim_octopus` to type at it, or run the whole ROS side against it:
`ros2 launch camera_path_ik preview.launch.py arm:=sim`. The `arm_bridge` tests also run
against it when it is built (`cd src/arm_bridge && python3 -m pytest test`).

## Flash

From any computer with Arduino IDE and STM32CubeProgrammer: copy the
`arduino/arm_firmware/` folder over and follow its
[`README.md`](arduino/arm_firmware/README.md) (board settings, libraries, BOOT0 + DFU upload).
The firmware starts at 0x08000000, so flashing replaces BTT's SD-card bootloader.

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

One per line, not case-sensitive. A leading number is a sequence number, echoed in the
reply; leave it out when typing (the reply then starts with `-`). Every request gets exactly
one reply line, `<seq> OK q=<free slots> ...` or `<seq> ERR <CODE> q=<free slots> ...`.

The protocol commands (`HELLO`, `LIMITS`, `ENABLE`, `DISABLE`, `SETPOS`, `SEG`, `GO`, `END`,
`STOP`, `STATUS`, `CLEAR`, `PING`) are specified in section 4 of the decisions doc.
Bring-up commands, only while IDLE:

| Command | What it does |
|---|---|
| `HELP` | List commands |
| `INFO` | Firmware version, CPU clock (should be 180000000 Hz), and `tick_max`: the longest step interrupt since boot, in CPU cycles. It must stay well under `tick_budget` (3600) during a fast move on every joint. |
| `DRIVERS` | Configures every driver and checks it answers. Needs motor power. Run again after motor power is cycled (drivers forget their settings). |
| `SPIN <joint> <steps> [rate]` | Move one joint at a constant rate (steps/s, default 400, no acceleration). Negative steps reverse. Type anything to abort. Needs `ENABLE`; clears `homed`. |
| `ENDSTOPS` | Raw endstop levels (pulled up: 1 = high, also what a disconnected sensor reads) |

`ENABLE` / `DISABLE` switch **all** drivers. **Z falls when disabled unless supported.**

Joints: `THETA Z R YAW PITCH`.

## First test, in order

1. USB only, no motor power: `PING`, then `INFO` (checks the clock setup: `sysclk=180000000`).
2. `ENDSTOPS` while blocking/unblocking each optical gate by hand: note which level means
   blocked, for D-24.
3. Motor power on: `DRIVERS`. Every driver should report its version (TMC5160 `0x30`,
   TMC2209 `0x21`). `NO ANSWER` = motor power off, wrong mode jumpers, or wrong slot.
4. One motor, joint free to move or disconnected from the mechanism:
   `ENABLE`, then `SPIN YAW 3200 400` (one turn at 16 microsteps, 200-step motor).
5. Z only with the carriage supported: `SPIN Z 400 200`. Check both motors turn the same
   way; if not, set `invert_dir` on Z2 in `src/config.h`.
6. The step generator, by hand: `HELLO`, `LIMITS` for all five joints (e.g.
   `LIMITS YAW -3200 3200 4000`), `SETPOS 0 0 0 0 0`, `SEG 200000 0 0 0 640 0`, `END`,
   `GO`: Yaw turns a fifth of a turn in 0.2 s, then `! DONE`. Keep hand-typed moves under
   0.25 s: the watchdog stops a move when nothing arrives from the PC for 250 ms.
7. Then from ROS: `ros2 launch camera_path_ik preview.launch.py arm:=serial port:=/dev/ttyACM0`.

The currents in `src/config.h` are low placeholders (400 mA / 300 mA) until the real motors
are known (D-14).
