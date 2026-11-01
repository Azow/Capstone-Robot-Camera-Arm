# camera_path_editor

Type in camera moves as pairs of points with a speed at each one, see them drawn
in RViz, and save or load them as files. Data entry and visualization only; the
arm itself is `camera_path_ik`.

A camera move is two paths built together, row by row:

- **Camera path**: where the camera body goes (drawn **blue**).
- **Focus path**: the point the camera aims and focuses at (drawn **red**).
- **Aim arrows** (drawn **green**, see-through): from every dense camera point
  to the matching focus point, showing where the camera looks along the move.

Row *n* means "the camera is at this blue point, looking at this red point."
The matching numbers on the blue and red spheres show which ones go together.

All coordinates are **meters** relative to the robot base (`base_link`):
x forward, y left, z up.

## Packages

| Package | What it is |
|---|---|
| `camera_path_interfaces` | Message definitions only (`PathRow`, `PathTable`, `PathIkStatus` for the IK column, and `PlaybackState` for the play controls). No code. |
| `camera_path_editor` | The panel, the path node, the launch file, and the RViz config. |

```
 path_panel  --PathTable on /camera_path/table-->  path_node  --MarkerArray on /camera_path/markers-->  RViz
 (you type)                                       (samples paths)                                      (you look)
```

## Build

```bash
cd ~/capstone_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select camera_path_interfaces camera_path_editor
source install/setup.bash
```

## Launch

```bash
ros2 launch camera_path_editor editor.launch.py
```

This opens two windows: the **Camera Path Editor** table and **RViz**. It does
not load the arm model. The arm (`arm_description_v2`) and its launch files are
separate and unchanged.

Optional: change the time between dense points (default 0.02 s, 50 per
second), and how often an aim arrow is drawn along them (default every 5th
point, so 10 arrows per second of motion):

```bash
ros2 launch camera_path_editor editor.launch.py sample_period:=0.1 arrow_every:=1
```

## Using the panel

The toolbar along the top holds the file and row controls on the left and the
display toggles on the right. The column headers are coloured like the RViz
drawing: camera columns blue, focus columns red.

- **Add** appends a row at the end, copying the last row's values.
- **Insert** adds a row directly below the selected row, copying it.
- **Delete**, **Up**, **Down** act on the selected row.
- Double-click a number to edit it. Press Enter or click away to apply.
  Anything that isn't a number is rejected and the old value is kept.
- The **−** and **+** buttons along the bottom of each number change it by
  0.01 m per click, for fine adjustments. Each click is applied right away.
- **Speed (m/s)** is the camera's speed as it passes that row. See
  [Speed and timing](#speed-and-timing). The first and last rows are always 0
  (greyed out). New rows get 0.05 m/s.
- Between each pair of rows is a shaded segment line (labelled `1→2` etc.)
  with a dropdown under the camera columns and one under the focus columns:
  `linear` (a straight line) or `arc` (a tangent arc, see below). The Speed
  column on that line shows how long the segment takes at 1x.
- Rows renumber automatically. Every change updates RViz immediately.
- The **Notes** column (far right) is filled in by `camera_path_ik` when it is
  running (`ros2 launch camera_path_ik preview.launch.py`). It stays empty
  when the whole path is reachable. Otherwise it shows, in red, why the first
  unreachable point can't be reached: on that row, or on the segment line
  between two rows if the point is partway along. Hover for the full text.
  It also shows speed problems and arc notes on segment lines. The status
  bar's right end says which path point is unreachable, or that all are.
- **Show: Aim arrows** turns the green arrows on and off. It is a display
  setting only: it is not saved in the file. It sets path_node's
  `show_aim_arrows` parameter, so path_node must be running.
- **Show: Keyframes** does the same for the waypoint spheres and their row
  numbers and the path lines between them (path_node's `show_keyframes`).
- The bar under the table holds the play controls. They are greyed out unless
  `camera_path_ik` is running (`preview.launch.py`): **Play/Pause**, a
  **Speed** dropdown (0.25x to 4x of the path's own timing), and a slider to
  scrub along the path, with the time into the path next to it.
  Dragging the slider pauses playback. See `camera_path_ik`'s README.
- **Save** (Ctrl+S) writes the table (rows, speeds and segment types) to a `.json` file.
  **Load** (Ctrl+O) replaces the current table with one from a file and redraws.
  The window title shows the file last saved or loaded. Loading does not ask
  about unsaved changes, so save first if you want to keep them.

### File format

One row per line, so files are easy to read and edit by hand:

```json
{
  "version": 2,
  "rows": [
    {"camera": [0.6, -0.4, 0.3], "focus": [0.0, 0.0, 0.2], "speed": 0.0},
    {"camera": [0.6, 0.0, 0.5], "focus": [0.0, 0.0, 0.2], "speed": 0.05},
    {"camera": [0.6, 0.4, 0.3], "focus": [0.0, 0.0, 0.2], "speed": 0.0}
  ],
  "camera_segment_types": ["linear", "linear"],
  "focus_segment_types": ["linear", "linear"],
  "s_curve": true
}
```

`s_curve` is optional (missing means off). A file with a broken row (not 3 numbers, a negative speed, not valid JSON) is
refused and the current table is left alone. Missing or unknown segment types
load as `linear`. Version 1 files (no `speed`) still load, with 0.05 m/s on
every middle row. The status line says what was changed.

Since Add copies the previous row, a new point appears exactly on top of the
previous one in RViz until you change its numbers.

## Testing without the panel

Run the node on its own and publish a table by hand:

```bash
ros2 run camera_path_editor path_node
```

```bash
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /camera_path/table camera_path_interfaces/msg/PathTable "{rows: [{camera: {x: 0.6, y: -0.4, z: 0.3}, focus: {x: 0.0, y: 0.0, z: 0.2}}, {camera: {x: 0.6, y: 0.0, z: 0.5}, focus: {x: 0.0, y: 0.0, z: 0.2}, speed: 0.05}, {camera: {x: 0.6, y: 0.4, z: 0.3}, focus: {x: 0.0, y: 0.0, z: 0.2}}], camera_segment_types: [linear, linear], focus_segment_types: [linear, linear]}"
```

Leave out the middle row's `speed` and every segment starts and ends at 0, so
nothing past row 1 is drawn (see [Speed and timing](#speed-and-timing)).
The node logs the dense point count on every update, e.g.
`Drew 3 rows: 651 points every 0.02 s (12.97 s), 131 aim arrows`.
An arrow is skipped wherever the camera sits exactly on its focus point.

Unit tests for the sampling math and the file format:

```bash
cd ~/capstone_ws/src/camera_path_editor && python3 -m pytest test
```

## Node parameters (`path_node`)

| Parameter | Default | Meaning |
|---|---|---|
| `sample_period` | 0.02 | Seconds between dense points. Must match `camera_path_ik`'s. Can be changed while running; the picture redraws. |
| `arrow_every` | 5 | Draw an aim arrow on every Nth dense point (and always on every row and the last point). 1 = an arrow on every point. Can be changed while running. |
| `frame_id` | `base_link` | Frame all markers are drawn in. |
| `show_aim_arrows` | `true` | Draw the green aim arrows. The panel's checkbox sets it; can be changed while running. |
| `show_keyframes` | `true` | Draw the waypoint spheres, row numbers and path lines. The panel's checkbox sets it; can be changed while running. |

## Speed and timing

Each row's **Speed** is how fast the camera moves as it passes that row. Within
a segment the speed changes at a constant rate from one row's speed to the
next, so:

- rising speed (0 → 0.1) is an **ease in**,
- equal speeds (0.1 → 0.1) are **linear**, constant speed,
- falling speed (0.1 → 0) is an **ease out**.

Speed never jumps at a row, because both segments that meet there use that
row's speed. The arm always starts and ends at rest (first and last rows are 0).
A segment of length L takes `2 L / (v_start + v_end)` seconds, shown on the
segment line. For "move, stop, move", set a middle row's speed to 0.

- **Pan only** (the camera doesn't move, the focus does): the focus path's
  length is used instead, so the speed applies to the focus point there.
- **Neither moves**: the segment takes no time.
- **Starts and ends at 0, and moves**: it would never get going, so it can't be
  timed. The segment line says *can't time* in red, with the reason in the IK
  column, and the path (drawing and playback) stops at the row before it. Give
  one end a speed, or add a row in between.

**S-curve** (checkbox in the bar under the table): by default the speed changes
at a constant rate within each segment, so acceleration jumps at rows (e.g.
from speeding up straight to cruising). With S-curve on, each segment's speed
change eases in and out instead, so acceleration is 0 at every row and changes
smoothly in between. Durations are exactly the same either way, so you can flip
it to compare. The cost is a 1.5x higher peak acceleration mid-segment. It's
part of the path (saved in the file as `"s_curve"`), so the drawing, IK and
playback all follow it. Changing it re-solves and goes back to the start.

Arcs still bend the path suddenly where they meet a line, which is a sideways
acceleration jump the S-curve doesn't remove. Direction is not smoothed yet: a sharp corner taken at speed still turns
instantly. Set the speed at a corner to 0 if that matters.

## Tangent arcs

An `arc` segment is a circular arc that leaves its start row in the direction
the path arrived there and ends exactly on the next row. Those three things fix
the circle, so there's nothing else to set (like SolidWorks' Tangent Arc).
Camera and focus paths each choose their own segment types.

- Arc after arc is smooth. A line after an arc usually has a corner, because
  the circle decides the direction the arc leaves in.
- An arc needs a moving segment before it (holds are skipped over to find
  one). On the first segment it's drawn as a line, with a note.
- If the next row is behind the direction of travel, the arc loops round more
  than half a circle and the Notes column says so.
- Its length (for timing) is measured along the curve.

## How the dense points are made

Points are spaced evenly in time, every `sample_period` seconds. Each segment
is split into a whole number of equal time steps (no longer than
`sample_period`), so every row lands exactly on a point; the final row is added
once at the end. Because they're evenly spaced in time, points (and aim arrows)
bunch up where the camera is slow and spread out where it's fast.

The camera and focus paths share one timeline: at every point both are the same
fraction of the way through their own segment, so point k of one always pairs
with point k of the other. The math lives in `camera_path_editor/sampling.py`,
which is where new segment types (curves) will be added.

## Changes from the V1 spec, and why

- **Two packages instead of three.** Messages must be built with CMake
  (`ament_cmake`), so they stay in their own package. The node and panel are
  both Python and live together in `camera_path_editor` to keep all tool code
  in one place.
- **Renamed** to drop the "shot" wording: packages `camera_path_interfaces` /
  `camera_path_editor`, messages `PathRow` / `PathTable`, topics
  `/camera_path/table` / `/camera_path/markers`.
- **Fixed frame is `base_link`**, not `world`, so paths are relative to the
  arm base. It is the `frame_id` node parameter.
- **Panel is Option B**: a standalone Python window (PyQt5 via
  `python_qt_binding`, already installed with ROS) instead of a C++ RViz
  plugin. The node and messages don't depend on this choice.
- **New rows copy** the selected or last row instead of starting at zero.
- **Changing `sample_period` at runtime redraws immediately**.
- **Segment types stay with their position when rows move.** Moving a row
  swaps the waypoints but not the dropdowns between them. With only `linear`
  this makes no visible difference; worth revisiting once curves exist.
- **The RViz config adds a grid and a small axes marker** at the base origin
  for orientation.
- The panel publishes an empty table when it starts, so RViz matches the
  empty panel instead of showing a leftover path from a previous session.
