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
                         Octopus Pro v1.1 firmware (new, firmware/)
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
├── firmware/                   <- PlatformIO project for the Octopus Pro (not built by colcon)
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
| D-02 | Toolchain: PlatformIO + Arduino framework (STM32duino) | Decided |
| D-03 | Firmware lives in `firmware/` at the repo root, outside `src/` | Decided |
| D-04 | PC side is a Python ROS 2 node, `arm_bridge`, in `src/` | Decided |
| D-05 | Board buffers timed move segments sent ahead by the PC | Decided |
| D-06 | Firmware works in motor steps; `arm_bridge` converts rad/m to steps | Decided |
| D-07 | Line-based text protocol with sequence numbers | Decided |
| D-08 | Host computer: the team's WSL laptop | Decided |

**D-01: Custom firmware with a serial protocol.**
micro-ROS only changes how the board talks to the PC; step generation, driver setup and
homing are the same work either way. For this board micro-ROS would also need the
STM32CubeMX/Docker build flow and a custom USB transport. A plain serial protocol can be
tested by hand from a serial terminal with no ROS running, which matters most during bring-up.
Klipper was rejected because its motion planner expects printer-style kinematics, not
coordinated moves of our 5 joints.
*Consequence:* keep all communication code in one firmware module (`protocol`) so it could be
swapped for micro-ROS later without touching motion code.

**D-02: PlatformIO + Arduino (STM32duino).**
Easiest to start with, runs in VS Code, and the `TMCStepper` library supports both the
TMC5160 and TMC2209.
*Consequence:* PlatformIO has no "Octopus Pro" board, so we build for the Nucleo-H723ZG (same
chip and package) and override the clock setup (25 MHz crystal, 400 MHz CPU) and pin map.
The firmware is linked after the stock 128 KiB SD-card bootloader (0x08020000), so it is
flashed by copying `firmware.bin` to an SD card. Setup details: `firmware/README.md`.

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

### Hardware: drivers, motors, power

| ID | Decision | Status |
|---|---|---|
| D-10 | Driver per joint: TMC5160T Pro on Theta, Z1, Z2, R; TMC2209 on Yaw, Pitch | Decided |
| D-11 | All drivers plug directly into Octopus Pro driver slots | Decided |
| D-12 | Z axis is driven by 2 motors on 2 separate drivers | Decided |
| D-13 | Power: 48 V for Theta/Z1/Z2/R, 24 V for Yaw/Pitch | Decided |
| D-14 | Motor models, currents and step angles | Open (later) |
| D-15 | Transmission per joint (gear ratio / belt / leadscrew) | Open (later) |
| D-16 | Driver slot assignment: Driver0–3 Theta/Z1/Z2/R, Driver4–5 Yaw/Pitch | Decided |
| D-17 | Z is driven by a belt and pulley system | Decided |

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
| Yaw | TBD | TBD | TBD | TBD | TBD | TBD per rad |
| Pitch | TBD | TBD | TBD | TBD | TBD | TBD per rad |

**D-17: Belt-driven Z.**
Belts can be back-driven: the weight of the carriage turns the motors when they aren't
holding. So Z **will fall** whenever the Z motors lose power or are disabled. See D-31.

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
| Driver6, 7 | spare | | | | STOP_2, 6, 7 spare (e-stop later) |

The 48 V drivers are grouped on Driver0–3 and the 24 V drivers on Driver4–5, so the power
jumpers are easy to check at a glance.

> Use only **Pro v1.1** pin maps. The non-Pro Octopus config differs: e.g. its Driver3 enable
> pin (PA0) is the HE0 heater output on the Pro v1.1. The firmware drives all heater and fan
> outputs off at boot as a precaution.

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

## 4. Firmware build order

Each step is testable on its own before the next. Status in brackets.

1. **Board bring-up:** USB serial, clock check (`INFO`). [code written, compiles; not yet
   tested on the board]. No LED blink: the Octopus status LED pin (PA13) is also the SWD
   debug pin.
2. **Driver communication:** read back the version register from every TMC5160 and
   TMC2209 (`DRIVERS`). Proves the jumpers and wiring before anything moves.
   [code written, not yet tested]
3. **One motor spins** from a serial command (`SPIN`). [code written, not yet tested]
4. **All axes in sync,** including both Z motors, through a timer interrupt and segment queue.
5. **Homing** with the optical switches, in the order Pitch, Yaw, Theta, Z, R.
6. **Safety:** `STOP`, watchdog, soft limits, e-stop input.
7. **`arm_bridge`** package and the WSL usbipd / udev setup.
8. **Connect `camera_path_ik`:** send timed points; RViz shows the real positions.

---

## 5. Open questions

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

---

## Change log

| Date | Change |
|---|---|
| 2026-11-01 | Created. Recorded architecture, driver/power, homing and safety decisions. |
| 2026-11-01 | Decided buffering (D-05), text protocol + options (D-07), belt Z (D-17), one Z switch (D-21), homing order (D-22), optical endstops (D-23). E-stop deferred (D-30). Z hold options updated for belt drive (D-31). |
| 2026-11-01 | Unit conversion on PC (D-06). Homing order changed to Pitch, Yaw, Theta, Z, R (D-26 supersedes D-22); Z homes up, R homes outward (D-25); rest-position startup (D-27); half-vane flags proposed (D-28). Z hold assigned to mechanical team. |
| 2026-11-01 | StallGuard added as a homing backup (D-29), then deferred: not in the first firmware. |
| 2026-11-01 | Slot assignment decided (D-16). Bring-up firmware added in `firmware/` (build steps 1–3, untested on hardware). |
