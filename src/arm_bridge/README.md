# arm_bridge

The link between ROS and the arm's Octopus Pro firmware (decisions D-04 to D-09 in
`docs/control_system_decisions.md`; the serial protocol is section 4 there).

- `arm_bridge` (node): connects to the board, sends it the step limits, runs paths, and
  publishes the board's position on `/joint_states` and its state on `/arm/status`.
- `fake_board`: a pretend board on a pseudo-terminal, so everything can be tested with no
  hardware.

## Try it without hardware

```
ros2 launch camera_path_ik preview.launch.py arm:=fake
```

An **Arm** row appears in the editor panel. Then:

1. **Motors on**.
2. **Home** (the fake board slowly moves every joint to its home position), or
   **Set home here** to skip homing.
3. **Run on arm**: the arm moves to the path's start, then plays the path. The solid arm in
   RViz is the board's position; the see-through ghost is the preview (Play / slider).
4. **Stop** (or Esc) stops it at any time.

The speed dropdown applies to the next Run.

## With the real board

```
ros2 launch camera_path_ik preview.launch.py arm:=serial port:=/dev/ttyACM0
```

(`/dev/octopus` once the udev rule is set up.) The firmware can't home yet (build step 5),
so use **Set home here** with the arm placed at its home position by hand.

## Config

`config/arm.yaml`: steps per rad/m, home position, soft limits, and speed/acceleration limits
per joint. **All placeholders** until the motors and transmissions are chosen. A path that is
too fast for a joint is refused with a message saying which point and joint.

## Talking to a board by hand

```
ros2 run arm_bridge fake_board          # creates /tmp/fake_octopus
picocom /tmp/fake_octopus               # or /dev/ttyACM0 for the real board
```

Type `HELLO`, `STATUS`, `PING`... (sequence numbers are optional when typing).

## Tests

```
cd src/arm_bridge && python3 -m pytest test
```

They run the bridge against the fake board over a real pseudo-terminal.
