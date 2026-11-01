"""Joint units <-> steps, and turning a timed joint path into board segments.

No ROS and no serial port here, so it can be tested on its own.
"""

import math
from dataclasses import dataclass

import yaml

from arm_bridge.protocol import JOINTS, MAX_SEGMENT_US

SEGMENT_PERIOD = 0.02    # s; the move to the start is sampled at this rate
MIN_MOVE_TIME = 0.2      # s; shortest move to the start
MERGE_BELOW_US = 5000    # a segment shorter than this is merged into the one before


class PlanError(Exception):
    """The path can't be run; the message says why, for the user."""


@dataclass
class JointConfig:
    name: str              # board name (protocol.JOINTS)
    urdf_name: str
    steps_per_unit: float
    home: float            # joint position at step 0, rad or m
    min: float
    max: float
    max_velocity: float
    max_accel: float

    @property
    def unit(self):
        return 'm' if self.name in ('Z', 'R') else 'rad'

    def to_steps(self, value):
        return round((value - self.home) * self.steps_per_unit)

    def to_units(self, steps):
        return self.home + steps / self.steps_per_unit

    def step_limits(self):
        lo, hi = sorted((self.to_steps(self.min), self.to_steps(self.max)))
        return lo, hi

    def max_rate(self):
        """Top speed for the board, steps/s: a margin over max_velocity, because rounding
        positions to whole steps makes short segments look slightly faster."""
        return math.ceil(self.max_velocity * abs(self.steps_per_unit) * 1.1 + 100)


class ArmConfig:

    def __init__(self, joints):
        names = tuple(j.name for j in joints)
        if names != JOINTS:
            raise ValueError(f'joints must be listed in the order {JOINTS}, got {names}')
        self.joints = joints

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = yaml.safe_load(f)
        return cls([JointConfig(**{k: (v if k in ('name', 'urdf_name') else float(v))
                                   for k, v in j.items()}) for j in data['joints']])

    @property
    def urdf_names(self):
        return [j.urdf_name for j in self.joints]

    def to_steps(self, values):
        return [j.to_steps(v) for j, v in zip(self.joints, values)]

    def to_units(self, steps):
        return [j.to_units(s) for j, s in zip(self.joints, steps)]

    def reorder(self, names, positions):
        """Positions given for `names` (URDF names, any order) -> board order."""
        index = {n: i for i, n in enumerate(names)}
        missing = [n for n in self.urdf_names if n not in index]
        if missing:
            raise PlanError(f'path has no position for {", ".join(missing)}')
        return [positions[index[n]] for n in self.urdf_names]


def _check_limits(config, q, where):
    for j, v in zip(config.joints, q):
        if not j.min - 1e-9 <= v <= j.max + 1e-9:
            raise PlanError(f'{where}: {j.urdf_name} = {v:.4f} {j.unit} is outside '
                            f'its limits [{j.min}, {j.max}]')


def move_time(config, start, end):
    """Time for a smooth move (smoothstep) from start to end, respecting every joint's
    max_velocity and max_accel. Smoothstep peaks at 1.5x the average speed and has a peak
    acceleration of 6 d / T^2."""
    t = MIN_MOVE_TIME
    for j, a, b in zip(config.joints, start, end):
        d = abs(b - a)
        t = max(t, 1.5 * d / j.max_velocity, math.sqrt(6 * d / j.max_accel))
    return t


def plan_move(config, start, end):
    """Points [(time, q)] of a smooth move, starting after `start` and ending at `end`."""
    duration = move_time(config, start, end)
    count = max(1, math.ceil(duration / SEGMENT_PERIOD))
    points = []
    for i in range(1, count + 1):
        s = i / count
        s = s * s * (3 - 2 * s)
        points.append((duration * i / count, [a + (b - a) * s for a, b in zip(start, end)]))
    return points


def plan_run(config, start_steps, times, positions):
    """Turn a timed path (board order, joint units) into segments.

    Moves from start_steps to the path's first point, then follows the path. Returns
    (segments, move_duration, path_duration), where each segment is (dur_us, steps).
    Raises PlanError if the path is outside the limits or too fast.
    """
    if not times:
        raise PlanError('there is no path to run (nothing reachable?)')
    for i, q in enumerate(positions):
        _check_limits(config, q, f'point {i + 1}')
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            continue
        for j, a, b in zip(config.joints, positions[i - 1], positions[i]):
            speed = abs(b - a) / dt
            if speed > j.max_velocity * 1.001:
                raise PlanError(
                    f'point {i + 1} at {times[i]:.2f} s: {j.urdf_name} needs '
                    f'{speed:.3f} {j.unit}/s, max is {j.max_velocity} '
                    f'(slow the path down or lower the speed)')

    start = config.to_units(start_steps)
    move = plan_move(config, start, positions[0])
    move_duration = move[-1][0]
    # The path's own times, shifted to start when the move ends.
    t0 = times[0]
    timed = move + [(move_duration + t - t0, q) for t, q in zip(times[1:], positions[1:])]

    segments = []
    last_us = 0
    for t, q in timed:
        t_us = round(t * 1e6)
        dur = t_us - last_us
        if dur <= 0:
            continue
        steps = config.to_steps(q)
        if dur < MERGE_BELOW_US and segments:
            prev_dur, _ = segments.pop()
            dur += prev_dur
        # A long hold or slow move: split so no segment exceeds the board's maximum.
        parts = math.ceil(dur / MAX_SEGMENT_US)
        if parts > 1:
            prev = segments[-1][1] if segments else list(start_steps)
            for k in range(1, parts):
                f = k / parts
                segments.append((dur // parts,
                                 [round(a + (b - a) * f) for a, b in zip(prev, steps)]))
            dur -= (dur // parts) * (parts - 1)
        segments.append((dur, steps))
        last_us = t_us
    return segments, move_duration, timed[-1][0] - move_duration
