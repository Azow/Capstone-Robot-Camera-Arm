"""Save and load path tables as JSON files.

Pure Python, no ROS or Qt imports, so it can be unit tested on its own.

File format (one row per line so it stays easy to read and hand-edit):

    {
      "version": 1,
      "rows": [
        {"camera": [0.6, -0.4, 0.3], "focus": [0.0, 0.0, 0.2]},
        {"camera": [0.6, 0.4, 0.3], "focus": [0.0, 0.0, 0.2]}
      ],
      "camera_segment_types": ["linear"],
      "focus_segment_types": ["linear"]
    }

Coordinates are meters relative to base_link, same as the panel.
"""

import json
import math

from camera_path_editor.sampling import SUPPORTED_SEGMENT_TYPES

FORMAT_VERSION = 1


class TableFileError(Exception):
    """The file can't be used as a path table. The message says why."""


def dumps_table(rows, camera_segs, focus_segs):
    """Return the file text for a table.

    rows: list of [cx, cy, cz, fx, fy, fz].
    """
    row_lines = [
        '    ' + json.dumps({'camera': list(r[:3]), 'focus': list(r[3:])})
        for r in rows
    ]
    rows_text = '[\n' + ',\n'.join(row_lines) + '\n  ]' if rows else '[]'
    return (
        '{\n'
        f'  "version": {FORMAT_VERSION},\n'
        f'  "rows": {rows_text},\n'
        f'  "camera_segment_types": {json.dumps(list(camera_segs))},\n'
        f'  "focus_segment_types": {json.dumps(list(focus_segs))}\n'
        '}\n'
    )


def save_table(path, rows, camera_segs, focus_segs):
    try:
        with open(path, 'w') as f:
            f.write(dumps_table(rows, camera_segs, focus_segs))
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
    """Check loaded JSON and return (rows, camera_segs, focus_segs, notes).

    Bad rows are an error. Missing, wrong-length, or unknown segment types are
    replaced with 'linear' and described in notes so the panel can say so.
    """
    if not isinstance(data, dict) or not isinstance(data.get('rows'), list):
        raise TableFileError('The file has no "rows" list.')

    rows = []
    for number, row in enumerate(data['rows'], start=1):
        if not isinstance(row, dict):
            raise TableFileError(f'Row {number} is not an object.')
        rows.append(_point(row, 'camera', number) + _point(row, 'focus', number))

    notes = []
    gaps = max(0, len(rows) - 1)
    camera_segs = _segments(data, 'camera_segment_types', gaps, notes)
    focus_segs = _segments(data, 'focus_segment_types', gaps, notes)
    return rows, camera_segs, focus_segs, notes


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
