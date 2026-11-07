# Control System Decisions

The decision log for the arm's motor control: the microcontroller firmware, the
motor drivers and power, and the ROS bridge that connects them to `camera_path_ik`.

**How to use this file**

- Every decision gets an ID (`D-##`) and a status:
  - **Decided**: agreed; build on it.
  - **Proposed**: recommended, waiting for team sign-off.
  - **Open**: not decided yet; see the notes for options.
- Don't delete a decision that changes. Mark it **Superseded by D-##** and add the new one,
  so we keep the history of *why*.
- Items marked **⚠ Verify** are facts about the hardware we believe are true but must check
  against the BTT Octopus Pro manual / pinout or the driver datasheet before relying on them.
- Add a line to the [Change log](#change-log) whenever you edit this file.

---

## 1. System overview

```
camera_path_editor ──PathTable──> camera_path_ik (IK + playback)
                                        │ joint commands (rad / m)
                                        ▼
                                  arm_bridge (new ROS 2 node, Python, src/)
                                        │ USB serial: timed move segments (steps)
                                        ▼
                       Octopus (STM32F446) firmware (new, firmware/)
                                        │ STEP/DIR + SPI/UART driver config
                                        ▼
                     6 drivers ──> 6 stepper motors (Theta, Z x2, R, Yaw, Pitch)
                                        │
                                        ▼ actual step positions reported back
                                  arm_bridge ──/joint_states──> robot_state_publisher ──> RViz
```

The PC does all the "smart" work (paths, IK, timing). The board only executes timed joint
moves safely: step generation, driver setup, homing, limits, stopping.

## 2. Repository layout

```
capstone_ws/
├── docs/                       <- this file and other design docs
├── firmware/                   <- Arduino sketch for the Octopus (not built by colcon)
└── src/                        <- ROS 2 packages (built by colcon)
    ├── arm_description_v2/
    ├── camera_path_editor/
    ├── camera_path_ik/
    ├── camera_path_interfaces/
    └── arm_bridge/             <- NEW: serial link between ROS and the firmware
```

---

## 3. Decisions

### Software architecture

| ID | Decision | Status |
|---|---|---|
| D-01 | Custom firmware with a serial protocol (not micro-ROS, not Klipper) | Decided |
| D-02 | Toolchain: Arduino IDE + STM32duino, one sketch file (was PlatformIO) | Decided (revised 2026-11-07) |
| D-03 | Firmware lives in `firmware/` at the repo root, outside `src/` | Decided |
| D-04 | PC side is a Python ROS 2 node, `arm_bridge`, in `src/` | Decided |
| D-05 | Board buffers timed move segments sent ahead by the PC | Decided |
| D-06 | Firmware works in motor steps; `arm_bridge` converts rad/m to steps | Decided |
| D-07 | Line-based text protocol with sequence numbers | Decided |
| D-08 | Host computer: the team's WSL laptop | Decided |
| D-09 | `ik_node` hands the bridge the whole timed path; the bridge streams it | Decided |

**D-01: Custom firmware with a serial protocol.**
micro-ROS only changes how the board talks to the PC; step generation, driver setup and
homing are the same work either way. For this board micro-ROS would also need the
STM32CubeMX/Docker build flow and a custom USB transport. A plain serial protocol can be
tested by hand from a serial terminal with no ROS running, which matters most during bring-up.
Klipper was rejected because its motion planner expects printer-style kinematics, not
coordinated moves of our 6 joints.
*Consequence:* keep all communication code in one firmware module (`protocol`) so it could be
swapped for micro-ROS later without touching motion code.

**D-02: Arduino IDE + STM32duino, one sketch file.**
The Arduino framework (STM32duino) is easy to start with, and the `TMCStepper` library
supports both the TMC5160 and TMC2209. The firmware started as a PlatformIO project with PC
unit tests and a simulator, copied into an Arduino sketch for flashing (D-34). On 2026-11-07
that was replaced by a single sketch, `firmware/arm_firmware/arm_firmware.ino` (plus
`build_opt.h` for the 12 MHz crystal), edited and flashed directly in Arduino IDE.
*Consequences:* STM32duino's generic F446ZE board with our own clock setup and pin map. No PC
build of the firmware any more: no C++ unit tests and no `arm:=sim`; the Python fake board
(`arm:=fake`) is the stand-in for the board. Setup details: `firmware/README.md`.

**D-03 / D-04: Repo layout.** colcon only builds folders with a `package.xml`; the firmware
needs a cross-compiler instead, so it lives next to `src/`. `arm_bridge` is Python to match
the existing packages (`pyserial` for the port).

**D-05: Buffered timed segments.**
The PC sends segments like "reach these joint positions in 50 ms" a few hundred ms ahead; the
board queues and plays them back to back.
*Why:* USB passthrough into WSL (usbipd) can pause for tens of ms. Without a buffer the arm
would stutter or jerk on every pause.
*Consequences:*
- Pressing Stop on the PC takes effect after what is already queued unless the firmware
  supports an immediate `STOP` that clears the queue and decelerates (it must, see D-32).
- `camera_path_ik` currently publishes positions at a fixed 20 points/s with no timing.
  It will need to send a duration (or timestamp) with each point.
- Flow control: the board reports how many free queue slots it has (in every reply), and
  `arm_bridge` never sends more than that. Otherwise the PC could overflow the queue.
- If the queue runs empty mid-motion (PC fell behind), the board decelerates to a stop and
  reports it, instead of jumping when the next segment arrives.

**D-06: Steps on the board, units on the PC.**
The firmware only knows steps, step limits and homing settings. `arm_bridge` holds
steps-per-radian / steps-per-meter per joint in a YAML config.
*Why:* motors and transmissions aren't chosen yet (D-14, D-15), so these numbers will change
often. Editing a config file and restarting a node is much faster than re-flashing. (The
board could easily do the math itself; this is about convenience, not processing load.)
*Example:* 200 steps/rev × 16 microsteps × 5:1 gearbox = 16,000 steps per joint turn
= 2,546 steps/rad, so Theta = 0.5 rad is sent as 1,273 steps.
*Consequences:*
- Commands typed by hand into a serial terminal are in steps.
- Soft limits are needed on the board for safety (in steps) and on the PC for IK (in rad/m).
  To keep one source of truth, `arm_bridge` sends the step limits to the board when it
  connects, and the board refuses motion until it has them.

**D-07: Line-based text protocol with sequence numbers.**
One command per line, plain text (`HOME`, `SEG ...`, `STATUS`, `STOP`, `ENABLE`), so anyone
can drive the board from a serial terminal. Each command carries a sequence number and the
board's reply echoes it, so `arm_bridge` knows which command an `OK` or `ERR` belongs to and
can detect a lost line. Exact commands to be defined in a protocol section.

*Why it's enough:* USB already checks every packet for corruption, and our traffic is tiny
(about 50 segments/s × ~60 bytes = 3 KB/s on a 12 Mbit/s link). Readability matters more
than compactness.

Options considered:

| Option | What it is | Pros | Cons |
|---|---|---|---|
| **Plain text lines** (chosen) | `SEG 12 50 1200 -340 ...\n` | Type and read by hand; any serial terminal works; trivial to parse in Python | Bigger messages; no checksum of its own (USB covers this) |
| Text + checksum | Same, with a checksum at the end of each line (how 3D printer firmware sends G-code) | Catches corruption on noisy links (e.g. raw UART, long cables) | Harder to type by hand; not needed over USB. Easy to add later if needed |
| JSON lines | `{"cmd":"seg","seq":12,"ms":50,"steps":[...]}` | Self-describing, easy to extend, `json` in Python, ArduinoJson on the board | Verbose, needs a JSON library on the board, still awkward to type |
| Binary packets | Packed structs, framed (e.g. COBS) with a CRC | Smallest and fastest, robust framing | Not human-readable; need a tool for every test; struct layouts must match exactly on both sides |
| Schema-based (Protocol Buffers via nanopb, MessagePack, CBOR) | Messages defined in a schema file, code generated for both sides | Versioned, typed, compact | Extra libraries and a code-generation step; overkill for ~6 commands |
| G-code | The 3D printer language (`G1 X10 Y20 F3000`) | Familiar, existing tools | Built for Cartesian moves and feed rates, not timed joint segments; we'd bend it to fit |
| Modbus RTU | Industrial register-based protocol | Standard, many tools | Built for reading/writing registers, poor fit for streaming motion |
| micro-ROS | Board speaks ROS directly | Native ROS topics | Ruled out by D-01 |

If the protocol module is kept separate (D-01), moving to binary or JSON later only changes
that module and the matching code in `arm_bridge`.

**D-08: Host is the WSL laptop.**
*Consequences:* the board's USB serial must be attached to WSL with `usbipd-win` every time
it is plugged in. Add a udev rule so it always appears under a fixed name (e.g. `/dev/octopus`).
A native Linux machine or Raspberry Pi can replace it later without firmware changes.

**D-09: The bridge gets the whole path, then streams it.**
`ik_node` publishes the solved path once, as a standard `trajectory_msgs/JointTrajectory`
(every point with its time), on `/camera_path/trajectory`. `arm_bridge` keeps the latest
one and, when told to run, streams it into the board's queue ahead of time (D-05).
*Why:* if the bridge only saw `ik_node`'s live 20 Hz poses it could never send ahead, and
any USB pause would empty the board's queue.
*Consequences:*
- The speed dropdown scales the whole path's timing (same proportions, faster or slower).
  `ik_node` applies it to the published trajectory; the bridge uses whatever was published
  when Run was pressed. Changing it during a run only affects the next run.
- Play, Pause and the slider stay preview-only. Running the real arm is a separate
  Run on arm / Stop arm control, so scrubbing never moves hardware.
- Before a path, the bridge moves the arm from where it is to the path's first point, timed
  from per-joint speed and acceleration limits in its config.
- With hardware (or the fake board) attached, RViz's main arm shows the board's reported
  position and the preview is drawn as a see-through ghost arm.
- There are no encoders. The "position" the board reports is the steps it has sent, not a
  measurement. Missed steps (stall, collision) are not detected (D-29 deferred).

### Hardware: drivers, motors, power

| ID | Decision | Status |
|---|---|---|
| D-10 | Driver per joint: TMC5160T Pro on Theta, Z1, Z2, R; TMC2209 on Yaw, Pitch, Focus | Decided |
| D-11 | All drivers plug directly into Octopus driver slots | Decided |
| D-12 | Z axis is driven by 2 motors on 2 separate drivers | Decided |
| D-13 | Power: 48 V for Theta/Z1/Z2/R, 24 V for Yaw/Pitch/Focus | Decided |
| D-14 | Motor models, currents and step angles | Open (later) |
| D-15 | Transmission per joint (gear ratio / belt / leadscrew) | Partly decided (Yaw, Pitch, Focus) |
| D-16 | Driver slot assignment: Driver0–3 Theta/Z1/Z2/R, Driver4–6 Yaw/Pitch/Focus | Decided |
| D-17 | Z is driven by a belt and pulley system | Decided |
| D-34 | Board: BTT Octopus with the STM32F446ZE; flashed from Arduino IDE with STM32CubeProgrammer | Decided |
| D-35 | Lens focus is a sixth motor axis, set from the camera-to-focus-point distance | Decided |
| D-37 | 1/16 microstepping with interpolation to 1/256 on every driver; SpreadCycle on Theta/Z/R, StealthChop on Yaw/Pitch/Focus | Decided (for now) |

**D-34: STM32F446 board, flashed from Arduino IDE.**
The team switched from the Octopus Pro v1.1 (STM32H723) to an Octopus with the STM32F446ZE
(180 MHz Cortex-M4, 12 MHz crystal). Its pins are the Octopus V1.1 / Pro V1.0 set, confirmed
by the team's driver test sketch that ran on it. The board is flashed from a different computer
with Arduino IDE and STM32CubeProgrammer (USB DFU with the BOOT0 jumper, or SWD).
*Consequences:*
- The firmware is the Arduino sketch `firmware/arm_firmware/` (D-02).
- The firmware starts at 0x08000000: flashing replaces BTT's SD-card bootloader.
- The 50 kHz step interrupt has 3600 CPU cycles per tick at 180 MHz (was 8000 on the H723).
  ⚠ Verify on the board that a full-speed move with all 7 drivers keeps up.
- ⚠ Verify: whether this board takes 48 V motor power (D-13). The Octopus Pro has a 60 V
  `MOTOR_POWER` input; the plain Octopus V1.1 is limited to about 28 V.

**D-10 / D-11: Drivers.**
TMC5160s are configured over SPI, TMC2209s over single-wire UART, both through the Octopus
slot pins.
*Consequences:*
- ⚠ Verify: each Octopus slot has mode jumpers under the driver (SPI vs UART). They must be
  set per slot to match the driver in it.
- 6 drivers used, out of the board's 8 slots: 2 spare.

**D-12: Dual-motor Z.**
Both Z motors get the exact same step commands from the firmware, so they move as one joint.
To ROS it is still a single `Z` joint.
*Consequences:*
- If one motor skips steps, the two sides go out of line. The carriage is stiff enough to
  keep itself aligned, so we use a single Z switch (see D-21).
- Both Z drivers must use the same current and microstep settings.
- See D-13: both are on 48 V.

**D-13: Mixed voltage.**
⚠ Verify: the Octopus Pro has a separate `MOTOR_POWER` input (up to 60 V) next to the main
power input, and a jumper per driver slot selecting which one powers that driver.
- Theta, Z1, Z2, R slots → jumper to `MOTOR_POWER` (48 V).
- Yaw, Pitch slots → jumper to main power (24 V).

> **⚠ Critical:** the TMC2209 is rated for about 29 V maximum. A TMC2209 in a slot jumpered
> to 48 V will be destroyed. Check every slot jumper before first power-on, and label the
> board.

⚠ Verify: the TMC5160T Pro's voltage rating. 48 V needs margin above it, because motors
decelerating push voltage back up onto the supply.

The main 24 V input also powers the board's logic (MCU), so cutting the 24 V supply resets
the board and loses its homed position. Matters for the e-stop (D-30).

**D-14 / D-15: Motors and transmissions.** To be filled in. Needed to compute
steps-per-unit, currents, and maximum speeds/accelerations.

| Joint | Motor | Rated current | Step angle | Microsteps | Transmission | Steps per unit |
|---|---|---|---|---|---|---|
| Theta | TBD | TBD | TBD | TBD | TBD | TBD per rad |
| Z1 / Z2 | TBD | TBD | TBD | TBD | TBD | TBD per m |
| R | TBD | TBD | TBD | TBD | TBD | TBD per m |
| Yaw | TBD | TBD | TBD | 16 | GT3 belt: 20-tooth motor pulley (McMaster 3764N128) to a 277-tooth ring, 13.85:1 | 7053.7 per rad |
| Pitch | 2x 4798N12 (one per side) | TBD | TBD | 16 | GT2 belts: 16-tooth motor pulleys (McMaster 3684N11) to 120-tooth pulleys on the shaft, 7.5:1 | 3819.7 per rad |
| Focus | NEMA 14, 25 mm | TBD | TBD | 16 | 0.8 module gears: 60-tooth brass (McMaster 2664N467) to the lens gear, ~108 teeth (estimated), ~1.8:1 | ~916.7 per rad of the ring |

Yaw, Pitch and Focus come from the CAD (tooth counts measured from the exported meshes) and
the McMaster part numbers; the Theta ring has ~355 teeth at 3 mm pitch (motor pulley TBD). ⚠
Verify: the lens gear's tooth count, and how the two Pitch motors are driven (one driver or
two, D-10).

**D-17: Belt-driven Z.**
Belts can be back-driven: the weight of the carriage turns the motors when they aren't
holding. So Z **will fall** whenever the Z motors lose power or are disabled. See D-31.

**D-35: Lens focus axis.**
The camera has a manual cine prime: a 7Artisans "Dream" T1.5 (the CAD shows the 35 mm; the
35/50/75 mm all have a 300° focus throw with hard stops and 0.8 module gears). A NEMA 14 on
the follow-focus mount turns the focus ring through a gear. There is no zoom.
*Consequences:*
- Focus is joint 6, `FOCUS`, on Driver6 (TMC2209, 24 V). Protocol v2 (section 4) adds it to
  every position list.
- The PC owns the lens model, like every other unit conversion (D-06):
  `camera_path_ik/config/lens.yaml` turns camera-to-focus-point distance into a ring angle
  (rad, 0 at the infinity stop). It starts as an ideal model (angle ∝ 1/distance) and is
  replaced by measured points once the real lens is calibrated; nothing else changes.
  `arm_bridge/config/arm.yaml` turns ring angle into motor steps (gear ratio).
- camera_path_ik sends focus as a joint named `Focus` alongside the URDF joints; it is not in
  the URDF.
- Points closer than the lens's minimum focus distance are clamped to the close stop and
  logged.
- ⚠ Verify: focal length (printed on the barrel), lens gear and pinion tooth counts, and the
  sensor plane's offset from the URDF camera frame.
- At T1.5 depth of field is a few cm at 1 m, so the arm model's camera position must be
  accurate at close range.

**D-37: Microstepping and chopper modes.**
Priorities: speed and torque first, then smoothness and noise (vibration shows in the footage).
- **1/16 microsteps on every driver.** The firmware sends at most 40000 steps/s per joint
  (50 kHz tick), which is 750 RPM at 1/16, about as fast as a stepper keeps useful torque.
  Finer settings would make the firmware the speed limit (1/32: 375 RPM). Microstepping
  level barely changes torque at speed.
- **Interpolation on** (`intpol`): each driver smooths every 1/16 step into 1/256 steps
  using the timing of the previous step. 1/256 smoothness at 1/16 step rates. It doesn't
  change the coil current, so torque is unchanged; the motor lags its step input by at most
  one 1/16 step.
- **SpreadCycle on Theta, Z, R** (TMC5160): full torque at all speeds and the best response
  to load changes; Z holds the arm against gravity. Hiss when stopped is electrical noise,
  not vibration.
- **StealthChop on Yaw, Pitch, Focus** (TMC2209): nearly silent and very smooth when slow,
  which is what the camera sees. It loses torque under hard acceleration and at high speed,
  and tunes itself on its first moves (`pwm_autoscale`), so make the first moves after power-up
  gentle. Focus homing by stall (D-36) needs StealthChop.
- Later, if Yaw/Pitch skip steps on fast moves: hybrid mode (StealthChop below a speed,
  SpreadCycle above, `TPWMTHRS`), with the switch-over set from a bench skip test.
- These match each chip's power-up default, so nothing changes from earlier firmware except
  interpolation, which is now set explicitly.

**D-16: Slot assignment.** Coded in `firmware/src/config.h`. Pins from Klipper's and BTT's
`generic-bigtreetech-octopus-pro-v1.1.cfg` (they agree), in `firmware/src/board_pins.h`.

| Slot | Joint | Driver | Supply | Mode jumper | Homing input |
|---|---|---|---|---|---|
| Driver0 | Theta | TMC5160T Pro | 48 V | SPI | STOP_0 (PG6) |
| Driver1 | Z1 | TMC5160T Pro | 48 V | SPI | STOP_1 (PG9), the one Z switch |
| Driver2 | Z2 | TMC5160T Pro | 48 V | SPI | none (follows Z1) |
| Driver3 | R | TMC5160T Pro | 48 V | SPI | STOP_3 (PG11) |
| Driver4 | Yaw | TMC2209 | 24 V | UART | STOP_4 (PG12) |
| Driver5 | Pitch | TMC2209 | 24 V | UART | STOP_5 (PG13) |
| Driver6 | Focus | TMC2209 | 24 V | UART | STOP_6 (PG14) = this driver's DIAG (stall), jumper **fitted** (D-36) |
| Driver7 | spare | | | | STOP_2, 7 spare (e-stop later) |

The 48 V drivers are grouped on Driver0–3 and the 24 V drivers on Driver4–6, so the power
jumpers are easy to check at a glance.

> Use the **Octopus V1.1 / Pro V1.0 (F446)** pin map (D-34), not the Pro v1.1 (H723) one:
> e.g. Driver3's enable pin is PA0 here, and PA2 (the Pro v1.1's Driver3 enable) is the HE0
> heater output. The firmware drives all heater and fan outputs off at boot as a precaution.

### Homing and position

| ID | Decision | Status |
|---|---|---|
| D-20 | Every axis homes to a physical switch (no sensorless homing) | Decided |
| D-21 | Z has a single homing switch | Decided |
| D-22 | Homing order: R, Z, Theta, Yaw, Pitch | Superseded by D-26 |
| D-23 | Switches are optical endstops (slotted optical gates) | Decided |
| D-24 | Optical endstop wiring and polarity | Proposed |
| D-25 | Homing direction: Z homes up, R homes outward; Theta/Yaw/Pitch defined later | Partly decided |
| D-26 | Homing order: Pitch, Yaw, Theta, Z, R | Decided |
| D-27 | Startup procedure: arm placed in its rest position before homing | Decided |
| D-28 | Long "half-vane" flags so each sensor tells which side of home it's on | Proposed |
| D-29 | StallGuard as a backup stop during homing | Deferred (later quality-of-life feature) |
| D-36 | Focus homes by stall detection into the lens's infinity stop (exception to D-20) | Proposed (homing not written yet) |

**D-20: Switches on every axis.**
Replaces the earlier idea of using StallGuard (sensorless) on Yaw/Pitch.
*Consequences:*
- ⚠ Verify: on the Octopus, "DIAG" jumpers connect each driver's stall output to the matching
  endstop pin. With physical switches on those pins, the DIAG jumpers must be **removed**.
- StallGuard is not used in the first firmware. It may be added later as a homing backup
  (D-29, deferred) or for collision detection.
- Endstop inputs needed: Theta, Z, R, Yaw, Pitch = 5 (6 with an e-stop input), out of
  8 on the board.

**D-21: One Z switch.**
The Z carriage is stiff enough to keep its two sides aligned, so both Z motors home together
to one switch. If the carriage is ever found twisted, revisit with a second switch so each
side can home separately.

**D-22: Homing order: R, Z, Theta, Yaw, Pitch. Superseded by D-26.**
Assumed homing pulled the arm in and down. Replaced once we decided the arm starts in its
rest position (D-27) and homes outward from it.

**D-23: Optical endstops.**
A slotted optical gate: an infrared beam across a slot, broken by a flag (a thin tab) fixed
to the moving part. No contact, so no wear, and the trigger point repeats well.
*Consequences:*
- An optical gate detects **one position**, it is not a physical stop. The flag can pass
  through it, so the firmware must stop as soon as the beam breaks. The ends of travel away
  from the switch are protected only by the firmware's soft limits (D-32).
- Sensitive to dust in the slot and to strong sunlight/IR. Keep slots clean and shaded.
- Precise homing: approach fast until the beam breaks, back off, then approach slowly and
  take the slow trigger point as zero.
- Theta: if it can rotate far enough for the flag to reach the gate from both sides,
  homing must always approach from the same side (D-25).

**D-24: Optical endstop wiring and polarity (proposed).**
Optical endstop modules have 3 wires (power, ground, signal), unlike 2-wire mechanical switches.
- ⚠ Verify: the voltage on the Octopus endstop connector's power pin (3.3 V or 5 V), that
  our modules work at that voltage, and that their signal won't exceed what the STM32 input
  accepts.
- Proposed polarity: "beam blocked" reads as triggered, with the input pulled up so that a
  **disconnected signal wire also reads as triggered**. A broken wire then stops the axis
  instead of letting it run past the switch. ⚠ Verify with our modules: which level they
  output when blocked.
- Firmware treats "triggered" outside of homing as a fault and stops.

**D-25: Homing direction per joint (partly decided).**
- **Z homes up.** Z rests at its bottom-out (lowest) position. Its gate is a short distance
  above that, so homing moves Z up until the beam breaks.
- **R homes outward.** R rests fully retracted; its gate is a short distance out, so homing
  extends R until the beam breaks.
- **Theta, Yaw, Pitch:** open. Which way does each move to find its gate, and can Theta reach
  its gate from both sides?

*Consequence:* Z and R home *away* from their hard stops, toward a gate. If the arm starts on
the far side of the gate (e.g. Z left partway up), moving "up to find the gate" would never
find it and would run on toward the other end. Handled by D-27 and D-28.

**D-26: Homing order: Pitch, Yaw, Theta, Z, R.**
The arm starts in its rest position (D-27), so homing works outward from the wrist:
1. Pitch.
2. Yaw.
3. Theta.
4. Z (moves up to its gate).
5. R (extends to its gate).

Each joint finishes homing before the next starts.

**D-27: Startup procedure.**
Before power-on/homing, the team places the arm in its rest position by hand: Z bottomed out,
R fully retracted, the wrist near zero. This is stable (Z sits on its bottom stop, so it
can't fall) and puts every joint a short, known distance from its gate.
*Firmware consequences:*
- Each joint's homing search distance is limited to a little more than the rest-to-gate
  distance. If the beam hasn't broken by then, homing stops with an error instead of
  driving to the far end.
- If a sensor already reads blocked when homing starts, the joint first moves back off it
  (only possible to do safely with D-28), then homes normally.

**D-28: Half-vane flags (proposed, for the mechanical team).**
Make each flag long enough to block the beam over *all* of the travel on one side of home
(a "half-vane"), instead of a thin tab. The sensor then reads blocked on one side of home and
clear on the other. At startup the firmware knows which way to move, even if the arm wasn't
put in its rest position, and Theta can't approach from the wrong side.
If not possible on some joint, that joint relies on D-27 alone.

```
Thin tab (usual):        clear  clear  clear [BLOCKED] clear  clear  clear
                         -> when "clear", you can't tell which side of home you're on

Half-vane:               BLOCKED BLOCKED BLOCKED | clear  clear  clear
                                                 ^ home = the edge
                         -> blocked: move one way; clear: move the other way
```
For a linear joint (Z, R) the vane is a long strip on the moving part that ends at home. For
a rotating joint (Theta, Yaw, Pitch) it's an arc, or a half-disc if the joint can turn a
full circle. The firmware always finishes its approach to the edge from the same side, so
zero is repeatable.

**D-29: StallGuard as a homing backup (deferred).**
Not in the first firmware: the startup procedure (D-27) and the short search distance make
it very unlikely to be needed. Notes kept for when it's added later.

While a joint searches for its gate, the firmware also watches the driver's StallGuard
reading. If the motor stalls against something (a hard stop, or an obstacle), homing stops
with an error. Together with the short search distance (D-27), this catches the "arm wasn't
in its rest position" case: e.g. Z started above its gate and runs up into its top stop.

The optical gate stays the only source of the zero position. StallGuard only stops the
search; it never sets zero.

*Consequences and limits:*
- StallGuard only works within a speed range: it's unreliable at very low speeds. The fast
  first pass of homing is protected; the slow second approach (only a few mm or degrees from
  the gate) relies on the search distance alone.
- It needs tuning per joint (a sensitivity threshold), and the result changes with speed,
  supply voltage and load. Z is loaded by gravity while moving up, so it will tune
  differently from the others. Tuning happens once motors and mechanics are final.
- The motor still bumps the stop before the stall is detected. Keep homing speed and current
  low enough that this is harmless.
- TMC5160 (Theta, Z1, Z2, R): StallGuard2, works in SpreadCycle mode. Firmware reads it over
  SPI (fast). Both Z drivers are checked; a stall on either stops Z.
- TMC2209 (Yaw, Pitch): StallGuard4, only works in StealthChop mode. Read over the UART
  link, which is slower. ⚠ Verify the polling rate is fast enough at homing speed.
- The drivers' DIAG (stall) pins can't be used: their jumpers are removed because the
  optical endstops occupy those endstop pins (D-20). So the firmware polls the registers
  instead of getting an interrupt.

**D-36: Focus homes sensorlessly (proposed).**
The lens has solid hard stops at both ends of its focus throw, and there's no room for an
optical gate on the ring. Focus homes by driving toward infinity at low current until the
TMC2209's StallGuard4 trips; that stop is ring angle 0.
*Consequences:*
- Exception to D-20 (switches on every axis), for this axis only.
- Driver6's DIAG jumper stays **fitted** so the stall output reaches STOP_6 as an input
  (D-29's polling concern doesn't apply). The TMC2209 is already in StealthChop, which
  StallGuard4 needs.
- Needs tuning on the real lens (`SGTHRS`, `TCOOLTHRS`, homing speed and current); ignore the
  first steps while accelerating. Approach targets from the same side to take up gear backlash.

### Safety

| ID | Decision | Status |
|---|---|---|
| D-30 | Physical e-stop button | Deferred (revisit before running at full power) |
| D-31 | Z must hold position at all times, including power loss | Decided (requirement); method owned by mechanical team |
| D-32 | Software stop layers: GUI stop, firmware `STOP`, watchdog | Proposed |
| D-33 | Board refuses moves until homed, and after any e-stop or fault | Proposed |

**D-30: Physical e-stop (deferred).**
Not designed yet; the team agrees it's worth considering. Leave a spare endstop input free
for it. The case for it:
A software stop and a power switch are not enough on their own:
- The software stop only works if the PC, WSL, USB link and firmware are all working. The
  cases where you need a stop most (frozen program, USB dropped, firmware bug) are the cases
  where it fails.
- A power switch is not designed or placed to be hit quickly; a mushroom e-stop button is.
- 48 V motors with leadscrews and a lifting Z can pinch and crush.

Proposed design, details to decide:
- A latching red mushroom e-stop that cuts **motor power** (48 V and 24 V to the motors),
  through a relay or contactor rated for DC if switching on the DC side.
- A second contact on the button wired to an Octopus input, so the firmware knows the
  e-stop was pressed, stops sending steps, and requires re-homing.
- Open problem: since main 24 V also powers the board logic (D-13), cutting it resets the
  board. Options: power the board logic from USB or a separate small supply, or accept a
  reset on e-stop (re-home anyway).
- Interacts with D-31: an e-stop removes motor power, so Z must not fall when it does.

**D-31: Z holds position (requirement).**
**Owner: mechanical team.** Firmware needs to know the outcome (e.g. a brake output to control).

A stepper only holds while energized, and Z is belt-driven (D-17), so belts can't hold it.
When the drivers are disabled, power fails, or an e-stop cuts power, **Z will fall** unless
something else holds it. Options:
- **Power-off brake** on the Z motors (or the pulley shaft): spring-applied, released by
  power. Holds whenever power is lost. Firmware releases it before moving. Most reliable;
  costs a part and a driver output.
- **Self-locking worm gearbox** between motor and pulley: can't be back-driven. Slower Z
  and a mechanical change.
- **Counterbalance** (counterweight, gas spring, constant-force spring): makes Z close to
  weightless so it drifts instead of drops. Doesn't guarantee hold, but helps the motors.
- **Rely on motor holding torque only:** holds only while powered. Acceptable for early
  bench testing with the carriage supported, not for the final arm.

*Firmware consequences either way:* never disable the Z drivers on idle; keep a hold current
on Z; apply the brake (if any) before disabling.

**D-32: Software stop layers (proposed).**
1. Stop button in the editor panel → `arm_bridge` sends `STOP`.
2. Firmware `STOP`: clear the segment queue and decelerate all axes to a stop.
3. Watchdog: if the board is moving and hears nothing from the PC for a set time
   (e.g. 250 ms, TBD), it decelerates to a stop.
4. Soft limits on the board: any segment outside the joint limits is rejected.

These are in addition to the physical e-stop, not a replacement.

**D-33: Homing required (proposed).**
After power-on, e-stop, or a limit/fault, the board rejects motion commands until `HOME`
completes.

---

## 4. Serial protocol (v2)

The commands between `arm_bridge` and the board (D-07). `arm_bridge/protocol.py` and the
firmware must both follow this section; the fake board (`arm_bridge/fake_board.py`)
implements it in Python so the bridge can be tested without hardware.

### Framing

- ASCII text, one message per line, ending in `\n` (`\r\n` also accepted). At most 120
  characters. Fields are separated by single spaces.
- **Request:** `<seq> <COMMAND> [args...]`. `seq` is a positive integer the PC increases by
  one per request. When typing by hand, `seq` can be left out; the reply then uses `-`.
- **Reply:** exactly one per request, echoing its `seq`:
  - `<seq> OK q=<free> [key=value...]`
  - `<seq> ERR <CODE> q=<free> [message]`

  `q` is the number of free segment slots in the queue, in every reply (D-05 flow control).
- **Event:** a line the board sends on its own: `! <EVENT> [key=value...]`. Events can
  arrive between any two replies.
- The bridge sends one request and waits for its reply before the next (stop-and-wait).
  At ~100 requests/s over USB that is plenty, and it keeps the code simple.

### Positions and segments

- Positions are absolute **steps** from home, as signed 32-bit integers, in joint order
  `THETA Z R YAW PITCH FOCUS`. Z is one value; the board steps both Z motors (D-12). v2
  added `FOCUS` (D-35); v1 had the first five.
- A segment is `SEG <dur_us> <theta> <z> <r> <yaw> <pitch> <focus>`: move every joint in a straight
  line (constant speed) from where the previous segment ended to these targets, over
  `dur_us` microseconds. The bridge sends the path's points (one every 20 ms) as segments.
- The board checks every segment when it arrives, against the previous segment's end:
  `1000 ≤ dur_us ≤ 1000000`, every target inside the joint's limits, and no joint faster
  than its `max_rate`. It does **not** check acceleration; the PC plans that.
- Segments only start moving after `GO`. Segments sent while running are appended.
- `END` marks the last segment of a run. When the queue empties after `END` the board
  reports `! DONE`. If it empties without `END`, the PC fell behind: the board ramps every
  joint down to a stop and reports `! UNDERRUN` (D-05).

### States

| State | Meaning |
|---|---|
| `IDLE` | Not moving. Accepts everything. |
| `RUNNING` | Playing queued segments. Accepts `SEG`, `END`, `STOP`, `STATUS`, `PING`. |
| `STOPPING` | Ramping down after `STOP`, an underrun or the watchdog. Then `IDLE`. |
| `HOMING` | Running the homing sequence (D-26). Accepts `STOP`, `STATUS`, `PING`. |
| `FAULT` | Something went wrong that needs `CLEAR`. Clears `homed`. |

Flags reported alongside: `limits` (limits received since `HELLO`), `homed`, `enabled`.
Motion (`SEG`, `GO`) needs all three.

### Commands

| Command | Does | OK reply adds |
|---|---|---|
| `HELLO` | Start of a session: stops any motion, clears the queue and the limits. Sent on connect. | `proto=2 fw=<version> queue=<slots> max_rate=<steps/s>` |
| `LIMITS <joint> <min> <max> <max_rate>` | Soft limits in steps and top speed in steps/s for one joint (D-06). Needed for all 6 joints before motion. `max_rate` can't exceed the board's `max_rate`. | |
| `ENABLE` / `DISABLE` | All drivers on / off. `DISABLE` clears `homed` (motors can move unpowered; Z falls, D-31). | |
| `HOME` | Starts homing (D-26, D-27). Replies at once; finishes with `! HOMED` or `! FAULT code=HOME_FAILED`. | |
| `SETPOS <theta> <z> <r> <yaw> <pitch> <focus>` | **Bench testing only.** Declares the current position and sets `homed`, for testing before homing works. IDLE only. | |
| `SEG <dur_us> <6 targets>` | Queue one segment. | |
| `GO` | Start playing the queue. | |
| `END` | No more segments in this run. | |
| `STOP` | Clear the queue and ramp down to a stop; `! STOPPED` when still. Never refused. | |
| `STATUS` | Report the state. | `state=<S> homed=0/1 enabled=0/1 limits=0/1 pos=<6 comma-separated> seg=<seq of the segment playing, or 0>` |
| `CLEAR` | `FAULT` → `IDLE`. | |
| `PING` | Nothing (keeps the watchdog fed). | |

Bring-up commands (`INFO`, `DRIVERS`, `ENDSTOPS`, `SPIN`) stay for hand testing in `IDLE`;
the bridge doesn't use them.

### Errors

| Code | When |
|---|---|
| `BAD_CMD` | Unknown command |
| `BAD_ARGS` | Wrong number of arguments or not a number |
| `BAD_STATE` | Not allowed in the current state (e.g. `SEG` while `HOMING`) |
| `NO_LIMITS` | Motion before `LIMITS` for every joint |
| `NOT_HOMED` | Motion before homing (D-33) |
| `DISABLED` | Motion with the drivers off |
| `QUEUE_FULL` | `SEG` with no free slot |
| `OUT_OF_LIMITS` | A segment target is outside a joint's limits |
| `TOO_FAST` | A segment needs a joint faster than its `max_rate` |
| `NOT_IMPLEMENTED` | Command exists in the spec but not in this firmware yet (`HOME` until step 5) |

### Events

| Event | Meaning |
|---|---|
| `! DONE` | Queue finished after `END`; now `IDLE`. |
| `! STOPPED` | Finished ramping down after `STOP`; now `IDLE`. |
| `! UNDERRUN` | Queue ran empty without `END`; ramped down; now `IDLE`. |
| `! WATCHDOG` | No message from the PC for 250 ms while moving; ramped down; now `IDLE`. |
| `! HOMED` | Homing finished. |
| `! FAULT code=<CODE>` | Entered `FAULT`. |

### Watchdog

While `RUNNING` or `HOMING`, any received line feeds the watchdog. The bridge polls
`STATUS` every 20 ms, so a frozen PC or a dropped USB link stops the arm within 250 ms
(D-32). The 250 ms is a starting value.

### Example run

```
> 1 HELLO                          < 1 OK q=64 proto=2 fw=0.4 queue=64 max_rate=40000
> 2 LIMITS THETA -20000 20000 8000 < 2 OK q=64     (and the other 5 joints)
> 7 ENABLE                         < 7 OK q=64
> 8 HOME                           < 8 OK q=64
                                   < ! HOMED
> 9 SEG 400000 1200 0 0 0 0 4000   < 9 OK q=63     (move to the path's start)
> 10 SEG 20000 1215 3 0 0 0 3990   < 10 OK q=62    (path points...)
> 40 GO                            < 40 OK q=34
> 41 STATUS                        < 41 OK q=34 state=RUNNING ... seg=9
   ... SEG / STATUS until the path is sent ...
> 900 END                          < 900 OK q=20
                                   < ! DONE
```

---

## 5. Firmware build order

Each step is testable on its own before the next. Status in brackets.

1. **Board bring-up:** USB serial, clock check (`INFO`). [code written, compiles; not yet
   tested on the board]. No LED blink: the Octopus status LED pin (PA13) is also the SWD
   debug pin. Ported to the F446 (D-34); compiles in Arduino IDE, not yet run on it.
2. **Driver communication:** read back the version register from every TMC5160 and
   TMC2209 (`DRIVERS`). Proves the jumpers and wiring before anything moves.
   [code written, not yet tested]
3. **One motor spins** from a serial command (`SPIN`). [code written, not yet tested]
4. **All axes in sync,** including both Z motors, through a timer interrupt and segment queue.
   [code written: protocol v1 and a 50 kHz Bresenham step generator, in
   `firmware/arm_firmware/arm_firmware.ino`. Unit-tested on the PC and run against
   `arm_bridge` while the firmware was a PlatformIO project; those PC builds were removed
   with D-02's revision. Not yet tested on the board]
5. **Homing** with the optical switches, in the order Pitch, Yaw, Theta, Z, R.
6. **Safety:** `STOP`, watchdog, soft limits, e-stop input.
7. **`arm_bridge`** package and the WSL usbipd / udev setup. [`arm_bridge` and the fake
   board written and tested against each other (`preview.launch.py arm:=fake`); udev rule
   not yet written]
8. **Connect `camera_path_ik`:** send timed points; RViz shows the real positions.
   [done against the fake board (and the since-removed firmware simulator): `ik_node` publishes
   `/camera_path/trajectory` (D-09), the editor panel has an Arm row, and a separate
   Manual jogging window moves one joint at a time (`/arm/move_joint`)]

---

## 6. Open questions

| # | Question | Related |
|---|---|---|
| Q1 | Motor models, currents, step angles | D-14 |
| Q2 | Transmission for each joint | D-15 |
| Q3 | How Z holds when unpowered (belt-driven): brake, worm gearbox, or counterbalance? (mechanical team) | D-31 |
| Q4 | Add a physical e-stop? If so, what does it cut, and how is the board logic powered? | D-30 (deferred) |
| Q5 | Can the mechanical team use half-vane flags? | D-28 |
| Q6 | Homing direction for Theta, Yaw, Pitch; can Theta reach its gate from both sides? | D-25 |
| Q7 | Optical endstop model, supply voltage, and output when blocked | D-24 |
| Q8 | Confirm the ⚠ Verify items against the Octopus Pro manual and driver datasheets | D-10, D-13, D-20, D-24 |
| Q9 | Which lens focal length do we have? Lens gear and motor pinion tooth counts? | D-35 |

---

## Change log

| Date | Change |
|---|---|
| 2026-11-01 | Created. Recorded architecture, driver/power, homing and safety decisions. |
| 2026-11-01 | Decided buffering (D-05), text protocol + options (D-07), belt Z (D-17), one Z switch (D-21), homing order (D-22), optical endstops (D-23). E-stop deferred (D-30). Z hold options updated for belt drive (D-31). |
| 2026-11-01 | Unit conversion on PC (D-06). Homing order changed to Pitch, Yaw, Theta, Z, R (D-26 supersedes D-22); Z homes up, R homes outward (D-25); rest-position startup (D-27); half-vane flags proposed (D-28). Z hold assigned to mechanical team. |
| 2026-11-01 | StallGuard added as a homing backup (D-29), then deferred: not in the first firmware. |
| 2026-11-01 | Slot assignment decided (D-16). Bring-up firmware added in `firmware/` (build steps 1–3, untested on hardware). |
| 2026-11-01 | Bridge gets the whole timed path (D-09). Serial protocol v1 specified (section 4). `arm_bridge` and fake board built; sections renumbered (build order is now 5, open questions 6). |
| 2026-11-01 | Build step 4 written (firmware `lib/arm_core`: protocol v1, step generator, ramp-down, watchdog), with PC unit tests and a simulator. Manual jogging window added. |
| 2026-11-05 | Board changed to the STM32F446 Octopus, flashed from Arduino IDE (D-34). Pin map, clock and TMC5160 sense resistor (0.075 Ω) updated. |
| 2026-11-07 | Firmware moved to one Arduino sketch, `firmware/arm_firmware/arm_firmware.ino` (D-02 revised); PlatformIO, the PC unit tests and the firmware simulator (`arm:=sim`) removed. |
| 2026-11-07 | Lens focus added as a sixth axis, `FOCUS` on Driver6 (D-35); serial protocol v2. Focus homing by stall detection proposed (D-36). |
| 2026-11-07 | Microstepping and chopper modes recorded (D-37); interpolation set explicitly in the firmware. |
| 2026-11-07 | Yaw, Pitch and Focus transmissions recorded from the CAD and McMaster parts (D-15); `arm.yaml` updated. |
