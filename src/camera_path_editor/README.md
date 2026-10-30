# camera_path_editor

Type in camera moves as pairs of points and see them drawn in RViz, and save or
load them as files. Data entry and visualization only. No robot motion, no timing.

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

Optional: change how many points are generated per segment (default 20), and
how often an aim arrow is drawn along them (default every 2nd point, so 10
arrows per segment):

```bash
ros2 launch camera_path_editor editor.launch.py points_per_segment:=5 arrow_every:=1
```

## Using the panel

- **Add row** appends a row at the end, copying the last row's values.
- **Insert row** adds a row directly below the selected row, copying it.
- **Delete row**, **Move up**, **Move down** act on the selected row.
- Double-click a number to edit it. Press Enter or click away to apply.
  Anything that isn't a number is rejected and the old value is kept.
- The **−** and **+** buttons along the bottom of each number change it by
  0.01 m per click, for fine adjustments. Each click is applied right away.
- Between each pair of rows there is a **camera segment** and a **focus
  segment** dropdown. In V1 the only choice is `linear` (a straight line).
- Rows renumber automatically. Every change updates RViz immediately.
- The **IK** column (far right) is filled in by `camera_path_ik` when it is
  running (`ros2 launch camera_path_ik preview.launch.py`). It stays empty
  when the whole path is reachable. Otherwise it shows, in red, why the first
  unreachable point can't be reached: on that row, or on the segment line
  between two rows if the point is partway along. Hover for the full text.
  The bottom right says which path point it is, or that all are reachable.
- **Show aim arrows** turns the green arrows on and off. It is a display
  setting only: it is not saved in the file. It sets path_node's
  `show_aim_arrows` parameter, so path_node must be running.
- **Show keyframes** does the same for the waypoint spheres and their row
  numbers and the path lines between them (path_node's `show_keyframes`).
- The bottom row holds the play controls. They are greyed out unless
  `camera_path_ik` is running (`preview.launch.py`): **Play/Pause**, a
  **Speed** dropdown (0.25x to 4x of `camera_path_ik`'s `playback_rate`), and
  a slider to scrub along the path, with the current point next to it.
  Dragging the slider pauses playback. See `camera_path_ik`'s README.
- **Save...** writes the table (rows and segment types) to a `.json` file.
  **Load...** replaces the current table with one from a file and redraws.
  The window title shows the file last saved or loaded. Loading does not ask
  about unsaved changes, so save first if you want to keep them.

### File format

One row per line, so files are easy to read and edit by hand:

```json
{
  "version": 1,
  "rows": [
    {"camera": [0.6, -0.4, 0.3], "focus": [0.0, 0.0, 0.2]},
    {"camera": [0.6, 0.0, 0.5], "focus": [0.0, 0.0, 0.2]}
  ],
  "camera_segment_types": ["linear"],
  "focus_segment_types": ["linear"]
}
```

A file with a broken row (not 3 numbers, not valid JSON) is refused and the
current table is left alone. Missing or unknown segment types load as `linear`,
and the status line says which ones were changed.

Since Add copies the previous row, a new point appears exactly on top of the
previous one in RViz until you change its numbers.

## Testing without the panel

Run the node on its own and publish a table by hand:

```bash
ros2 run camera_path_editor path_node
```

```bash
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /camera_path/table camera_path_interfaces/msg/PathTable "{rows: [{camera: {x: 0.6, y: -0.4, z: 0.3}, focus: {x: 0.0, y: 0.0, z: 0.2}}, {camera: {x: 0.6, y: 0.0, z: 0.5}, focus: {x: 0.0, y: 0.0, z: 0.2}}, {camera: {x: 0.6, y: 0.4, z: 0.3}, focus: {x: 0.0, y: 0.0, z: 0.2}}], camera_segment_types: [linear, linear], focus_segment_types: [linear, linear]}"
```

The node logs the dense point counts on every update, e.g.
`Drew 3 rows (N=10): camera path 21 points, focus path 21 points, 21 aim arrows`.
An arrow is skipped wherever the camera sits exactly on its focus point.

Unit tests for the sampling math and the file format:

```bash
cd ~/capstone_ws/src/camera_path_editor && python3 -m pytest test
```

## Node parameters (`path_node`)

| Parameter | Default | Meaning |
|---|---|---|
| `points_per_segment` | 20 | Points per segment (N). Can be changed while running; the picture redraws. |
| `arrow_every` | 2 | Draw an aim arrow on every Nth point of each segment (counted from each waypoint, and always on the last point). 1 = an arrow on every point. Can be changed while running. |
| `frame_id` | `base_link` | Frame all markers are drawn in. |
| `show_aim_arrows` | `true` | Draw the green aim arrows. The panel's checkbox sets it; can be changed while running. |
| `show_keyframes` | `true` | Draw the waypoint spheres, row numbers and path lines. The panel's checkbox sets it; can be changed while running. |

## How the dense points are made

Each segment produces exactly N points at u = 0, 1/N, ..., (N-1)/N, and the
final waypoint is added once at the end, so each path has `(rows - 1) * N + 1`
points. The camera path and focus path always use the same N and the same u
values, so point k of one always pairs with point k of the other, even when one
segment is much longer than the other. Timing will rely on this later. The math
lives in `camera_path_editor/sampling.py`, which is where new segment types
(splines) will be added.

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
- **Changing `points_per_segment` at runtime redraws immediately**, so the
  N=5 check can be done without restarting.
- **Segment types stay with their position when rows move.** Moving a row
  swaps the waypoints but not the dropdowns between them. With only `linear`
  this makes no visible difference; worth revisiting once curves exist.
- **The RViz config adds a grid and a small axes marker** at the base origin
  for orientation.
- The panel publishes an empty table when it starts, so RViz matches the
  empty panel instead of showing a leftover path from a previous session.
