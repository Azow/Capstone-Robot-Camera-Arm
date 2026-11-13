"""The files arm_bench keeps: the results record (CSV) and each joint's test range (YAML).

results.csv has one row per test you marked, so you can look back at which speeds and
accelerations were acceptable and which skipped. Speeds and accelerations are stored both
in joint units (deg or mm, using arm.yaml's steps_per_unit at the time) and in motor steps,
which stay true even if a gear ratio in arm.yaml turns out to be wrong.
"""

import csv
import os

import yaml

FIELDS = ('time', 'joint', 'result', 'speed', 'accel', 'unit', 'travel', 'cycles', 'pause_s',
          'speed_steps_s', 'accel_steps_s2', 'motor_rev_s', 'travel_steps', 'steps_per_unit',
          'note')
ACCEPTABLE = 'acceptable'
SKIPPED = 'skipped'


def load_results(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def append_result(path, row):
    """Add one row; writes the header if the file is new."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS, extrasaction='ignore')
        if new:
            writer.writeheader()
        writer.writerow(row)


def load_ranges(path):
    """{joint name: upper end, steps from the lower end}."""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {name: int(v['upper_steps']) for name, v in data.items()
            if isinstance(v, dict) and 'upper_steps' in v}


def save_ranges(path, ranges):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write('# arm_bench test ranges: the upper end of each joint, in motor steps from\n'
                '# the lower end (where the marker is). Written by the tool.\n')
        yaml.safe_dump({name: {'upper_steps': steps} for name, steps in ranges.items()}, f,
                       sort_keys=False)
