# arm_bench

Bench test tools for the arm's motors: **motor_test**, a window with two tests:

- **Speed / accel test**: each joint's top speed and acceleration before it skips steps.
- **Focus stall test**: the focus ring's throw between its hard stops, measured with
  StallGuard, against what arm.yaml predicts. Needs the **bench firmware build**
  (`#define BENCH_TESTS 1` in arm_firmware.ino).

There are no encoders, so skipped steps are found with a **visual marker**: every test runs
out-and-back cycles and ends where it started. If the marker still lines up, no steps were
lost. A skip loses 4 full steps at once (7.2° at the motor shaft), so it shows, and it adds
up over the cycles.

## Run it

```
colcon build --packages-select arm_bridge arm_bench && source install/setup.bash
ros2 run arm_bench motor_test --port /dev/ttyACM0
```

No other ROS nodes are needed: it talks to the board over serial directly, using
arm_bridge's code. To try it without hardware, start the fake board first:

```
ros2 run arm_bridge fake_board            # then pick /tmp/fake_octopus as the port
```

Options: `--port`, `--config` (default: arm_bridge's `arm.yaml`), `--data-dir` (default
`~/arm_bench`).

## Using it

1. **Connect**. **Enable all** / **Disable all** check that every driver turns on and off
   (it configures every driver and lists the ones that didn't answer). Z drops when
   unpowered, so park it at the bottom first.
2. **Power only the joint you test** (bench firmware). The **Power** row has an on/off
   button per joint (green = on) and **Power this joint only**. With **Only the selected
   joint** ticked (the default), picking a joint in the list powers it and switches the
   others off, once anything is on. It always asks before switching Z off ("Keep Z on" keeps
   Z holding while you test another joint). A joint that was switched off has lost its
   position: it shows "unknown" until you **Set lower here** for it. The run firmware
   can't do this: the Power row is greyed out and only Enable all / Disable all work.
3. **Pick a joint** and get it to the **lower end** of the range you want to test: jog it
   there, or switch that joint off, move it by hand, and switch it on again. Line your
   marker up, then **Set lower here** (this position becomes step 0).
4. **Jog** to the other end and **Set upper here**. The upper end can be either side of
   the lower one. Ranges are saved, so next time you only redo step 3 at the same marker.
5. Set **top speed**, **acceleration** (the same rate speeding up and slowing down: a
   symmetric trapezoid), **cycles** and the **pause** at each end, then **Run test**. The
   steps/s and motor rev/s readouts show what the motor actually sees.
6. Check the marker and click **Acceptable** or **Skipped** (add a note if you like). If it
   skipped, realign to the marker and Set lower here again before the next test.

**Esc** or the orange button stops at any time. A test you stopped can still be marked; its
note says it stopped early.

**Space** (or the red button) is the **emergency stop**: it stops whatever is moving and then
disables every driver. It works anywhere in the window and never "clicks" the focused
button. The one exception is typing a command in the Console box while nothing moves.
Disabled motors go limp (Z drops), and the board forgets where they are. This is a software
stop: it needs USB and the firmware to be working, so keep a hand near the power switch.

## Focus stall test

Checks `steps_per_unit` for FOCUS (and so the guessed lens gear tooth count). arm.yaml
predicts the ring's whole throw is (max - min) × steps_per_unit = 300° × 916.73 = 4800
motor steps.

1. In the tab, click **Focus off** and turn the ring by hand onto one hard stop. Then click
   **Power focus only** (FOCUS on, every other motor off; it asks first if Z is on) and
   **Set zero here**. On this tab the Power row's "only" button and "Only the selected
   joint" act on FOCUS, not the joint picked in the speed test.
2. Set **Direction** toward the other stop. The labels assume `invert_dir` for FOCUS is
   right in the sketch; check which way it really turns.
3. Set **speed** (ring °/s; the steps/s are shown), **stall threshold** (SGTHRS 0..255; it
   trips when SG_RESULT ≤ 2 × threshold), **current** (mA; low keeps a stall gentle on the
   lens, default 180 of the 450 run current) and **max travel** (it stops there if nothing
   trips; default 1.2 × the predicted throw).
4. **Start seek**. The graph shows StallGuard live; the dashed line is the trip level and
   the shaded start is the 100 ms the firmware ignores. It stops on a stall, at max travel,
   or on Esc / Space.
5. The summary shows measured vs predicted steps, the error, and the steps/rad and lens
   tooth count that the measurement implies. **Go to zero** turns back to repeat it.

Picking a threshold: threshold 0 never trips. Do a short seek in the middle of the throw
(small max travel) and read the free-running StallGuard value off the graph, then pick a
threshold so 2 × threshold sits well below it. Stalling far too early = threshold too high.
StealthChop tunes itself on its first moves after power-up, so jog a little before the
first seek.

Every finished seek (not aborted ones) goes in `stall_results.csv`, shown in the tab's table.

**Console / log** tab: type board commands as in a serial terminal (`SPIN THETA 3200 800`,
`ENDSTOPS`, `INFO`, `HELP`). SPIN blocks the window until it finishes and makes the board
forget its position.

## What it saves

In `--data-dir` (`~/arm_bench`):

- `results.csv`: one row per marked test: time, joint, result, speed and acceleration in
  joint units (deg or mm) and in motor steps/s, steps/s², motor rev/s, travel, cycles,
  pause, the `steps_per_unit` used, and your note. The Results tab shows it, newest first.
  The steps columns stay true even if a gear ratio in `arm.yaml` turns out to be wrong.
- `ranges.yaml`: each joint's upper end, in steps from its lower end.
- `stall_results.csv`: one row per focus stall seek: settings, result (stall / limit),
  steps, measured and predicted throw, error %, implied steps/rad and lens teeth, lowest
  StallGuard reading.

## Limits while testing

The tool opens the board's soft limits wide and allows up to 38,000 steps/s (the firmware
caps at 40,000). The range you set is what keeps tests in bounds, and **jogging is not
limited**, so watch the hard stops (especially FOCUS, whose lens has hard stops at both ends
of its 300° throw). `arm.yaml` is only used for steps per unit and the default jog speed.
