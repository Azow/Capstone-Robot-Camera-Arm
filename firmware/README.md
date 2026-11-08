# Arm firmware (Octopus, STM32F446ZE)

Firmware for the BigTreeTech Octopus with the STM32F446ZE (Octopus V1.1 / Pro V1.0 pin set)
that drives the arm's stepper motors.
Design decisions are in [`../docs/control_system_decisions.md`](../docs/control_system_decisions.md).

**Status: protocol v1 and the step generator (build step 4), untested on hardware.** The
board speaks the serial protocol (decisions doc, section 4) that `arm_bridge` uses, queues
timed segments, and steps every joint from a 50 kHz timer interrupt. The bring-up commands
(`INFO`, `DRIVERS`, `ENDSTOPS`, `SPIN`) are still there for testing by hand. Homing (step 5)
is not written yet: `HOME` answers `NOT_IMPLEMENTED`; use `SETPOS` on the bench.

## Files

The whole firmware is one Arduino sketch, `arm_firmware/`:

| File | What it is |
|---|---|
| `arm_firmware/arm_firmware.ino` | All the firmware. Section 1 (Settings) is which joint is on which slot, driver type, current and microsteps; the comment at the top lists the other sections. |
| `arm_firmware/build_opt.h` | Compiler flag Arduino IDE has no menu for: the board's 12 MHz crystal. Keep it next to the `.ino`. |

## Slot assignment (D-16)

| Slot | Joint | Driver | Supply | Mode jumpers | Endstop |
|---|---|---|---|---|---|
| Driver0 | spare | | | | STOP_0 spare (e-stop later) |
| Driver1 | Focus | TMC2209 | **24 V** | UART | STOP_1 = its DIAG (stall), jumper fitted |
| Driver2 | Pitch | TMC2209 | **24 V** | UART | STOP_2 |
| Driver3 | Yaw | TMC2209 | **24 V** | UART | STOP_3 |
| Driver4 | R | TMC5160T Pro | 48 V | SPI | STOP_4 |
| Driver5 | Z2 | TMC5160T Pro | 48 V | SPI | none (STOP_5 spare) |
| Driver6 | Z1 | TMC5160T Pro | 48 V | SPI | STOP_6 (the Z switch) |
| Driver7 | Theta | TMC5160T Pro | 48 V | SPI | STOP_7 (DIR is PA14 = SWD clock) |

## Before the first power-on

Check all of these against the BTT Octopus manual for this board.

- [ ] **Driver1, 2 and 3 (TMC2209) are powered from 24 V, NOT the 48 V motor input.**
      A TMC2209 on 48 V is destroyed.
- [ ] This board takes 48 V motor power at all (the Octopus Pro has a 60 V `MOTOR_POWER`
      input; the plain Octopus V1.1 is limited to about 28 V). See D-34.
- [ ] Driver4–7 (TMC5160T Pro) are jumpered to the 48 V motor-power input, and the
      TMC5160T Pro is rated for 48 V with margin.
- [ ] Each slot's mode jumpers match its driver: SPI for Driver4–7, UART for Driver1–3.
- [ ] DIAG jumpers removed on every slot except Driver1 (Focus), which keeps its DIAG
      jumper: focus will home by stall (D-36).
- [ ] Driver orientation in the sockets (EN pin to EN): a reversed driver is destroyed.
- [ ] `kRSense5160` in the `.ino`'s Settings section (0.075 Ω, from BTT's wiki) matches the sense resistors
      on the TMC5160T Pro (marked R075; an R022 board needs 0.022). If the code's value is
      higher than the real one, the motors get more current than asked: 3.4x for R022.
- [ ] The Z carriage is supported: Z falls whenever its drivers are disabled (D-31).

## Build and flash (Arduino IDE)

### One-time setup

1. Arduino IDE: File > Preferences > Additional boards manager URLs, add
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
2. Boards Manager: install **STM32 MCU based boards** (by STMicroelectronics).
3. Library Manager: install **TMCStepper** (by teemuatlut), version 0.7.3.
4. Install **STM32CubeProgrammer** (Arduino IDE calls it to flash).

### Tools menu

| Setting | Value |
|---|---|
| Board | STM32 MCU based boards > Generic STM32F4 series |
| Board part number | Generic F446ZETx |
| USB support | CDC (generic 'Serial' supersede U(S)ART) |
| Upload method | STM32CubeProgrammer (DFU), or (SWD) with an ST-Link |

The build stops with an error if the board or USB setting is wrong, or if `build_opt.h` is
missing.

### Flashing over USB (DFU)

Open `arm_firmware/arm_firmware.ino` (or copy the `arm_firmware/` folder to the computer that
flashes), then:

1. Motor power off. Fit the BOOT0 jumper, connect USB-C, press reset.
2. Sketch > Upload.
3. Remove the BOOT0 jumper and press reset.

The firmware starts at 0x08000000, so flashing replaces BTT's SD-card bootloader.

### Testing without the board

There is no PC build of the firmware. To test the ROS side without hardware, use the
Python fake board, which follows the same protocol:
`ros2 launch camera_path_ik preview.launch.py arm:=fake`.

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
picocom --echo --imap lfcrlf /dev/ttyACM0   # or Arduino IDE's Serial Monitor, line ending Newline
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

Joints: `THETA Z R YAW PITCH FOCUS` (FOCUS is the lens focus motor on Driver1, D-35).

## First test, in order

1. USB only, no motor power: `PING`, then `INFO` (checks the clock setup: `sysclk=180000000`).
2. `ENDSTOPS` while blocking/unblocking each optical gate by hand: note which level means
   blocked, for D-24.
3. Motor power on: `DRIVERS`. Every driver should report its version (TMC5160 `0x30`,
   TMC2209 `0x21`). `NO ANSWER` = motor power off, wrong mode jumpers, or wrong slot.
4. One motor, joint free to move or disconnected from the mechanism:
   `ENABLE`, then `SPIN YAW 3200 400` (one turn at 16 microsteps, 200-step motor).
5. Z only with the carriage supported: `SPIN Z 400 200`. Check both motors turn the same
   way; if not, set `invert_dir` on Z2 in the Settings section.
6. The step generator, by hand: `HELLO`, `LIMITS` for all six joints (e.g.
   `LIMITS YAW -3200 3200 4000`), `SETPOS 0 0 0 0 0 0`, `SEG 200000 0 0 0 640 0 0`, `END`,
   `GO`: Yaw turns a fifth of a turn in 0.2 s, then `! DONE`. Keep hand-typed moves under
   0.25 s: the watchdog stops a move when nothing arrives from the PC for 250 ms.
7. Then from ROS: `ros2 launch camera_path_ik preview.launch.py arm:=serial port:=/dev/ttyACM0`.

The currents in the Settings section are about 70-75% of each motor's rated current (D-14).
