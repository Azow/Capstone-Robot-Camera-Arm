"""Single-joint moves for bench tests, turned into board segments.

Every move is a symmetric trapezoid: speed up at the chosen acceleration, cruise at the
chosen speed, slow down at the same acceleration. A move too short to reach the speed is a
triangle (speeds up to sqrt(accel * distance), then slows down). Everything here is in motor
steps; no serial port and no Qt, so it can be tested on its own.
"""

import math

from arm_bridge.protocol import MAX_SEGMENT_US, MIN_SEGMENT_US

SEGMENT_US = 10000   # the ramps are sampled every 10 ms; the board steps evenly within each


def trapezoid(distance, speed, accel):
    """Times of a symmetric trapezoid move over `distance` steps (sign ignored).

    Returns (accel_time, cruise_time, peak_speed): the move takes 2 * accel_time + cruise_time.
    """
    d = abs(distance)
    if speed <= 0 or accel <= 0:
        raise ValueError('speed and acceleration must be positive')
    if d * accel < speed * speed:          # never reaches `speed`: a triangle
        peak = math.sqrt(accel * d)
        return peak / accel, 0.0, peak
    return speed / accel, (d - speed * speed / accel) / speed, speed


def trapezoid_position(t, distance, speed, accel):
    """Steps travelled after t seconds (signed like distance)."""
    ta, tc, peak = trapezoid(distance, speed, accel)
    d = abs(distance)
    total = 2 * ta + tc
    if t <= 0:
        s = 0.0
    elif t < ta:
        s = 0.5 * accel * t * t
    elif t < ta + tc:
        s = 0.5 * accel * ta * ta + peak * (t - ta)
    elif t < total:
        s = d - 0.5 * accel * (total - t) ** 2
    else:
        s = d
    return math.copysign(s, distance) if distance else 0.0


def _split(total_us, count):
    """total_us as `count` whole-microsecond parts that add up exactly."""
    base, extra = divmod(total_us, count)
    return [base + (1 if i < extra else 0) for i in range(count)]


class SegmentBuilder:
    """Builds the segments of a sequence of moves and pauses of one joint.

    The other joints are held at exactly their start steps.
    """

    def __init__(self, start_steps, joint):
        self.joint = joint
        self._held = list(start_steps)
        self.position = start_steps[joint]
        self.segments = []      # (dur_us, steps)
        self.duration_us = 0

    def _add(self, dur_us, position):
        steps = list(self._held)
        steps[self.joint] = position
        self.segments.append((dur_us, steps))
        self.duration_us += dur_us

    def move_to(self, target, speed, accel):
        """A trapezoid move to `target` steps at `speed` steps/s and `accel` steps/s^2."""
        target = int(target)
        distance = target - self.position
        if distance == 0:
            return
        ta, tc, _ = trapezoid(distance, speed, accel)
        total_us = max(MIN_SEGMENT_US, round((2 * ta + tc) * 1e6))
        count = max(1, min(math.ceil(total_us / SEGMENT_US), total_us // MIN_SEGMENT_US))
        start = self.position
        # The board stretches a very short move to MIN_SEGMENT_US; scale time to match.
        scale = (2 * ta + tc) / (total_us / 1e6)
        elapsed = 0
        for i, dur in enumerate(_split(total_us, count)):
            elapsed += dur
            if i == count - 1:
                position = target
            else:
                position = start + round(
                    trapezoid_position(elapsed / 1e6 * scale, distance, speed, accel))
            self._add(dur, position)
        self.position = target

    def hold(self, seconds):
        """Stay still for `seconds` (still segments, so the board keeps running)."""
        total_us = round(seconds * 1e6)
        if total_us < MIN_SEGMENT_US:
            return
        for dur in _split(total_us, math.ceil(total_us / MAX_SEGMENT_US)):
            self._add(dur, self.position)

    @property
    def duration(self):
        return self.duration_us / 1e6


def cycle_test(start_steps, joint, upper, speed, accel, cycles, pause,
               approach_speed, approach_accel):
    """The out-and-back test: (if not there already) move to 0 at the approach speed, then
    `cycles` times 0 -> upper -> 0 at the test speed and acceleration, pausing `pause` s at
    each end. Ends at 0, where the marker is. Returns the SegmentBuilder."""
    b = SegmentBuilder(start_steps, joint)
    if b.position != 0:
        b.move_to(0, approach_speed, approach_accel)
        b.hold(pause)
    for i in range(cycles):
        b.move_to(upper, speed, accel)
        b.hold(pause)
        b.move_to(0, speed, accel)
        if i < cycles - 1:
            b.hold(pause)
    return b
