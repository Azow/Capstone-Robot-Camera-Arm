# camera_path_ik

Inverse kinematics for the camera arm: put the camera on each point of the
camera path (blue) while it looks at the matching point of the focus path
(red), and play the arm along the path in RViz.

The solver is closed-form (geometry, no iteration). It reads the arm's
dimensions from the URDF, so it fits any arm with the same joint layout:

```
Theta (turns about a vertical axis) -> Z (slides vertically) -> R (slides horizontally)
  -> Yaw (turns about a vertical axis) -> Pitch (turns about a horizontal axis) -> camera
```

Roll about the viewing axis is not controlled; the arm has no joint for it.
No timing, velocity limits, ros2_control or MoveIt yet: playback just steps
through the path points at a fixed rate.

## Files

| File | What it is |
|---|---|
| `kinematics.py` | Forward kinematics read straight from the URDF. Matches robot_state_publisher's TF to ~1e-15 m. |
| `arm_model.py` | Measures the arm (axes, offsets, reach) from the forward kinematics and checks the joint layout. |
| `ik.py` | The closed-form IK. One target -> joint positions, or the reason it can't be reached. |
| `ik_numeric.py` | A slow numerical solver that knows nothing about the layout. Only the tests use it, as a cross-check. |
| `ik_node.py` | ROS node: PathTable in, JointState (for RViz) and PathIkStatus (for the editor) out. |
| `launch/preview.launch.py` | Editor + arm + IK + camera view + RViz. |

The math files have no ROS imports, so the tests run with plain pytest.

```
 path_panel --PathTable--> ik_node --JointState--> robot_state_publisher --TF--> RViz (arm moves)
                              |
                              +--PathIkStatus on /camera_path/ik_status--> path_panel (IK column)
 path_panel --SetBool on /camera_path/set_playing, UInt32 on /camera_path/seek, `speed` parameter--> ik_node
 ik_node --PlaybackState on /camera_path/playback--> path_panel (play controls)
```

## Build and run

```bash
cd ~/capstone_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select camera_path_interfaces camera_path_editor camera_path_ik
source install/setup.bash
ros2 launch camera_path_ik preview.launch.py
```

This opens the editor table and RViz with the arm. Load or type a path, then press
**Play** at the bottom of the editor: the arm plays along it once and stops at
the end (Play again starts over). **Pause** holds the arm where it is. The
**Speed** dropdown next to it changes the speed without restarting, and the
slider scrubs to any point (dragging it pauses). Any edit to the path re-solves
it and goes back to the first point, paused. `examples/demo_path.json`
is a path the current arm can fully reach (a rising sweep in front of the red
pillar, watching it); load it with the editor's **Load...** button. The **Camera View** panel
shows the scene from the arm's camera: if the IK is right, the red focus sphere
stays in the middle of it.

Optional launch arguments:

- `points_per_segment:=10`: same meaning as in the editor; both use the same value (default 20).
- `arrow_every:=1`: how often the editor draws an aim arrow (default every 2nd point).
- `playback_rate:=10.0`: path points played per second at 1x speed (default 20, so one
  segment per second). Change it together with `points_per_segment` to keep the same speed.
- `free_spin:=true`: ignore the URDF's Theta and Yaw limits, so they can spin
  freely, any number of turns. Off by default: the arm keeps every URDF joint
  limit. The other joints always keep their limits.

The arm is `arm_description_v2`'s `camera_arm_v2.urdf`. `describe_arm` prints
what the solver measured:

```bash
ros2 run camera_path_ik describe_arm ~/capstone_ws/src/arm_description_v2/urdf/camera_arm_v2.urdf
```

For the current URDF: the wrist (Yaw axis) reaches 0.025 to 0.415 m from the
base axis, the camera height ranges from -0.089 to 0.661 m, and the camera sits
0.085 m off the Yaw axis.

## Unreachable points

Points are solved in order. At the first point the arm can't reach, solving
stops and the arm plays up to the point before it. The editor's **IK** column
shows the reason in red on that row, or on the segment line between two rows
if the point is partway along; hover over it for the full text. The bottom
right of the editor says which point it is ("IK: point 5 of 21 is
unreachable"), or "IK: all 31 points reachable". RViz shows only the arm and
the paths, no errors. The terminal logs the same reason, for example:

```
Point 1 of 21 (row 1) is unreachable: camera out of reach (wrist would be 0.5325 m from
the Theta axis, reach is 0.0251 to 0.4151 m). Nothing to play; showing the reference pose.
```

Possible reasons: too high or too low, out of reach, too close to the base
axis (the R slide passes through it on the current URDF, but R=0 stops the
wrist 2.5 cm out, so nothing closer can be reached), Pitch outside its limits, or Theta/Yaw outside
their limits.

## Node parameters (`ik_node`)

| Parameter | Default | Meaning |
|---|---|---|
| `urdf_path` | `arm_description_v2`'s `camera_arm_v2.urdf` | Arm to measure. |
| `points_per_segment` | 20 | Must match the editor's `path_node` so both use the same points. |
| `playback_rate` | 20.0 | Path points played per second at 1x. |
| `speed` | 1.0 | Multiplier on `playback_rate`. The editor's Speed dropdown sets it. |
| `free_spin` | false | Ignore the Theta and Yaw limits. Read once at startup. |

All but `urdf_path` and `free_spin` can be changed while running. `points_per_segment` re-solves
the path (back to the first point, paused); `playback_rate` and `speed` only change the timing.

## Testing

```bash
cd ~/capstone_ws/src/camera_path_ik && python3 -m pytest test
```

- `test_arm_model.py`: the measured model reproduces the URDF forward kinematics;
  measured numbers on a small hand-checkable test arm; each broken layout is rejected.
- `test_ik.py`: round trips (random pose -> camera point -> IK -> same camera
  point and aim) on three arms; smooth joint motion along a path; the singular
  cases; every unreachable reason; stopping at a joint limit instead of spinning
  a full turn; and a cross-check that the numerical solver
  never reaches a point the closed form rejects.

The test arms are in `test/arms.py`. One of them has the R slide passing
through the base axis and reaching it, so the Theta/Yaw singularity is tested
even though the current URDF stops 2.5 cm short of it.

## How the IK works

### 1. Measuring the arm (`arm_model.py`)

At one reference pose (all joints at 0), the forward kinematics reports where
each joint's axis sits in `base_link`:

| Symbol | Field | Meaning |
|---|---|---|
| `k` | `up` | Theta axis direction ("up") |
| `c` | `base_point` | a point on the Theta axis |
| `a_z`, `a_r` | `z_dir`, `r_dir` | Z and R slide directions |
| `y0`, `u_y` | `yaw_point`, `yaw_axis` | Yaw axis; `u_y = s k` with `s = +-1` |
| `p_p`, `u_p` | `pitch_point`, `pitch_axis` | Pitch axis |
| `P0`, `L0` | `camera_point`, `camera_look` | camera position and look direction (optical z) |

Measuring also checks the layout (Z and Yaw parallel to Theta, R and Pitch
perpendicular to it, within 0.001 rad) and raises `ArmLayoutError` otherwise.

With these, the arm can be moved without the URDF: apply each joint's motion
to the reference camera, from the camera end inward (Pitch, Yaw, R, Z, Theta),
about or along its axis as measured. That is `camera_pose()`, the "product of
exponentials" form of forward kinematics, and the tests check it equals the
URDF forward kinematics. The IK runs this formula backwards.

### 2. The forward formula, rearranged

Write `dq` for each joint's change from the reference pose, `Rot(u, a)` for a
rotation by `a` about direction `u`, and `h(v) = v - (v.k) k` for the
horizontal part of a vector. Then

```
L = Rot(k, dTheta) Rot(u_y, dYaw) Rot(u_p, dPitch) L0
P = c + Rot(k, dTheta) [ (y0 - c) + dZ a_z + dR a_r + Rot(u_y, dYaw) H(dPitch) ]

H(dPitch) = (p_p - y0) + Rot(u_p, dPitch) (P0 - p_p)        (the head offset)
```

The Yaw axis is parallel to the Theta axis, so the two rotations add into a
single heading `psi = dTheta + s dYaw`:

```
(1)  L = Rot(k, psi) Rot(u_p, dPitch) L0
(2)  P = c + Rot(k, dTheta) w + Rot(k, psi) H(dPitch),     w = (y0 - c) + dZ a_z + dR a_r
```

(1) involves only heading and pitch, so it is solved first; then everything
in (2) except Theta, Z and R is known.

Inputs: camera target `P*`, focus `F*`, aim `d = (F* - P*) / |F* - P*|`.

### 3. The steps (numbered the same in `ik.py`)

1. **Pitch.** Rotating about `k` doesn't change how much a vector points up, so
   `k . Rot(u_p, dPitch) L0 = k . d`. Splitting `L0` into a part along `u_p` and
   a part `m` across it gives `A cos(dPitch) + B sin(dPitch) = k.d` with
   `A = k.m`, `B = k.(u_p x m)`, so `dPitch = atan2(B, A) +- acos(k.d / sqrt(A^2+B^2))`.
   Two answers (normal and "over the top"); usually one is inside the limits.
2. **Heading.** `psi` is the angle about `k` from `h(Rot(u_p, dPitch) L0)` to `h(d)`.
   Undefined when looking straight up or down: Yaw is held (previous heading kept).
3. **Wrist.** `W = P* - c - Rot(k, psi) H(dPitch)`: where the wrist must be.
4. **Z.** Height is unchanged by Theta and R is horizontal, so
   `dZ = (k.W - k.(y0 - c)) / (k.a_z)`.
5. **R.** Distance from the axis is unchanged by Theta, so `|g + dR a_r| = |h(W)|`
   with `g = h(y0 - c)`. Solving the quadratic:
   `dR = -(g.a_r) +- sqrt(|h(W)|^2 - e^2)`, where `e` is how far the R slide passes
   from the axis. No answer if `|h(W)| < e`. With a slide that passes through the
   axis both answers can be valid: that is "slide through the middle instead of
   flipping Theta 180 deg".
6. **Theta.** The angle about `k` from `g + dR a_r` to `h(W)`. Undefined when the
   wrist is on the axis: Theta is held at its previous value.
7. **Yaw.** `dYaw = s (psi - dTheta)`. When Theta is held, Yaw covers the whole heading.
8. **Choose and check.** Theta and Yaw are only known up to a full turn. Along a
   path they must continue smoothly from the previous point (no full-turn jumps);
   if that passes a limit, the point is unreachable. For the first point, any
   turn inside the limits is allowed. Out of all valid combinations
   (2 pitch x 2 R), the closest to the previous point wins (0.1 m of slide counts
   as 1 rad). The answer is run through `camera_pose()`; position and aim must be
   within 1e-4 m and 0.05 deg, or it is reported as a failed self-check.

If the camera target equals its focus point there is no aim: Pitch and heading
are kept from the previous point and only the position is solved.

The closed form is exact apart from the URDF's rounding. CAD exports that write
1.5708 for pi/2 leave the Yaw axis ~1e-5 rad off parallel, enough to fail the
tests' 1e-5 rad aim check, so `camera_arm_v2.urdf` uses full-precision angles.
On the current URDF, 2000 random round trips land within 1e-14 m and 2e-6 deg,
at about 0.3 ms per point.

### Singularities

- **Wrist on the Theta axis** (the real arm fully retracted): Theta and Yaw turn
  about the same line, so only their sum matters. Theta is held. Near (not on)
  the axis, Theta has to turn fast: about `sideways camera speed / distance
  from axis` rad/s. That will matter once timing is added.
- **Looking straight up or down:** heading has no meaning. Yaw is held.
- The current URDF can't quite reach the first one (the slide passes through
  the axis, but R=0 stops the wrist 2.5 cm out, so Theta turns fast near it), and
  its Pitch limits (+-89.95 deg) stop just short of straight up and down. The
  test arms cover both.

## Known limits / next steps

- No timing: `time_from_start`, velocity and acceleration limits. The URDF has
  placeholder `effort="10"` / `velocity="1"` on every joint, and the Z limits are
  placeholders too; real limits are needed before timing.
- Joint limits are the only constraint. No collision checking (e.g. the pillar).
- Playback publishes `/joint_states` directly. The next step toward hardware is
  sending a `JointTrajectory` to ros2_control's `joint_trajectory_controller`
  (mock hardware first).
- When a point is unreachable, nothing after it is solved, even if later points
  are fine.
- With `free_spin:=false`, Theta and Yaw are limited to +-3.14 rad, so the
  camera can't circle past the +x side of the base (Theta = +-180 deg). A path
  that needs to stops there with "Theta/Yaw would pass its limit" instead of
  spinning a full turn back. With `free_spin:=true` there is
  no such stop, and nothing tracks how many turns the cables could take.
