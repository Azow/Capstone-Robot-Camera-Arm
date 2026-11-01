"""Save and load path tables as JSON files.

Pure Python, no ROS or Qt imports, so it can be unit tested on its own.

File format (one row per line so it stays easy to read and hand-edit):

    {
      "version": 2,
      "rows": [
        {"camera": [0.6, -0.4, 0.3], "focus": [0.0, 0.0, 0.2], "speed": 0.0},
        {"camera": [0.6, 0.0, 0.4], "focus": [0.0, 0.0, 0.2], "speed": 0.05},
        {"camera": [0.6, 0.4, 0.3], "focus": [0.0, 0.0, 0.2], "speed": 0.0}
      ],
      "camera_segment_types": ["linear", "linear"],
      "focus_segment_types": ["linear", "linear"],
      "s_curve": true
    }

Coordinates are meters relative to base_link, same as the panel. "speed" is
the camera speed at that row in m/s; the first and last rows are always 0.
Version 1 files (no "speed") still load: every middle row gets DEFAULT_SPEED.
"s_curve" (optional, default false) turns on S-curve speed changes.
"""

import json
import math

from camera_path_editor.sampling import (
    DEFAULT_SPEED, normalize_speeds, SUPPORTED_SEGMENT_TYPES,
)

FORMAT_VERSION = 2


class TableFileError(Exception):
    """The file can't be used as a path table. The message says why."""


def dumps_table(rows, speeds, camera_segs, focus_segs, s_curve=False):
    """Return the file text for a table.

    rows: list of [cx, cy, cz, fx, fy, fz]. speeds: one per row, m/s.
    """
    row_lines = [
        '    ' + json.dumps({'camera': list(r[:3]), 'focus': list(r[3:]), 'speed': v})
        for r, v in zip(rows, normalize_speeds(speeds))
    ]
    rows_text = '[\n' + ',\n'.join(row_lines) + '\n  ]' if rows else '[]'
    return (
        '{\n'
        f'  "version": {FORMAT_VERSION},\n'
        f'  "rows": {rows_text},\n'
        f'  "camera_segment_types": {json.dumps(list(camera_segs))},\n'
        f'  "focus_segment_types": {json.dumps(list(focus_segs))},\n'
        f'  "s_curve": {json.dumps(bool(s_curve))}\n'
        '}\n'
    )


def save_table(path, rows, speeds, camera_segs, focus_segs, s_curve=False):
    try:
        with open(path, 'w') as f:
            f.write(dumps_table(rows, speeds, camera_segs, focus_segs, s_curve))
    except OSError as e:
        raise TableFileError(f'Could not write the file: {e.strerror}.') from e


def load_table(path):
    """Read a table file. See parse_table for what is returned."""
    try:
        with open(path) as f:
            data = json.load(f)
    except OSError as e:
        raise TableFileError(f'Could not read the file: {e.strerror}.') from e
    except json.JSONDecodeError as e:
        raise TableFileError(f'Not valid JSON (line {e.lineno}): {e.msg}.') from e
    return parse_table(data)


def parse_table(data):
    """Check loaded JSON and return (rows, speeds, camera_segs, focus_segs, s_curve, notes).

    Bad rows are an error. Missing, wrong-length, or unknown segment types are
    replaced with 'linear', and missing speeds with DEFAULT_SPEED, and described
    in notes so the panel can say so.
    """
    if not isinstance(data, dict) or not isinstance(data.get('rows'), list):
        raise TableFileError('The file has no "rows" list.')

    rows = []
    speeds = []
    missing_speeds = 0
    for number, row in enumerate(data['rows'], start=1):
        if not isinstance(row, dict):
            raise TableFileError(f'Row {number} is not an object.')
        rows.append(_point(row, 'camera', number) + _point(row, 'focus', number))
        if 'speed' in row:
            speed = row['speed']
            if not _is_number(speed) or speed < 0:
                raise TableFileError(f'Row {number} "speed" must be a number >= 0.')
            speeds.append(float(speed))
        else:
            speeds.append(DEFAULT_SPEED)
            missing_speeds += 1
    speeds = normalize_speeds(speeds)

    notes = []
    # Only the middle rows' speeds matter; the ends are always 0.
    if missing_speeds and len(rows) > 2:
        notes.append(f'rows without a speed were set to {DEFAULT_SPEED} m/s')
    gaps = max(0, len(rows) - 1)
    camera_segs = _segments(data, 'camera_segment_types', gaps, notes)
    focus_segs = _segments(data, 'focus_segment_types', gaps, notes)
    s_curve = data.get('s_curve', False)
    if not isinstance(s_curve, bool):
        raise TableFileError('"s_curve" must be true or false.')
    return rows, speeds, camera_segs, focus_segs, s_curve, notes


def _point(row, key, number):
    value = row.get(key)
    if (not isinstance(value, list) or len(value) != 3
            or not all(_is_number(v) for v in value)):
        raise TableFileError(f'Row {number} "{key}" must be a list of 3 numbers.')
    return [float(v) for v in value]


def _is_number(value):
    # bool is a subclass of int in Python, so rule it out explicitly.
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _segments(data, key, gaps, notes):
    segs = data.get(key)
    if not isinstance(segs, list) or len(segs) != gaps:
        if gaps:
            notes.append(f'{key} was missing or the wrong length, set to linear')
        return ['linear'] * gaps

    result = []
    for i, seg in enumerate(segs):
        if seg in SUPPORTED_SEGMENT_TYPES:
            result.append(seg)
        else:
            notes.append(f'unknown {key[:-len("_segment_types")]} segment type '
                         f'"{seg}" at {i + 1}→{i + 2}, set to linear')
            result.append('linear')
    return result
