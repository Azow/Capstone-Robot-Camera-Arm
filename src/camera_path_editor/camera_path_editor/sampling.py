"""Turn a table of waypoints and speeds into a timed, dense point list.

Pure Python, no ROS imports, so it can be unit tested on its own.

Segment shapes (one per gap between rows, chosen separately for the camera
and focus paths):

- 'linear': a straight line to the next row.
- 'arc': a tangent arc. A circular arc that leaves its start row in the same
  direction the path arrived there, and ends on the next row. The start
  point, that direction and the end point fix the circle completely (like
  SolidWorks' Tangent Arc), so it needs no extra numbers. Its exit direction
  follows from the circle, so an arc after an arc is smooth, but a line after
  an arc usually has a corner. An arc needs a segment before it that moves
  (holds are skipped over); without one it's drawn as a line, with a note.

Timing model:

- Every row has a camera speed (m/s). The first and last rows are always 0:
  the arm starts and ends at rest.
- Within a segment the camera speeds up or slows down at a constant rate,
  from the speed at its start row to the speed at its end row. Rising speed
  is an ease in, equal speeds are linear, falling speed is an ease out.
  Speed is continuous across rows by construction.
- So a segment of length L (along the curve) takes T = 2 L / (v0 + v1) seconds.
- S-curve (optional, `s_curve=True`): instead of a constant acceleration, the
  speed change within each segment follows a smoothstep, so acceleration is 0
  at every row and changes smoothly in between: no acceleration jumps
  anywhere. It covers the same distance in the same time, so durations don't
  change; the peak acceleration mid-segment is 1.5x the constant one.
- If the camera doesn't move in a segment (a pure pan), the focus path's
  length is used instead, so the speed applies to the focus point there.
- If neither moves, the segment takes no time.
- A segment that moves but starts and ends at speed 0 can't be timed (it
  would never get going). Sampling stops at the start of the first such
  segment and reports it, the same way IK stops at an unreachable point.

The camera and focus paths share one timeline: at any moment both are the
same fraction of the way (by distance along the curve) through their own
segment.

Points are taken every `period` seconds within each segment. Each segment is
split into a whole number of equal steps (no longer than `period`), so every
waypoint is exactly on a point, and point k of the camera list always pairs
with point k of the focus list.
"""

import bisect
import math
from dataclasses import dataclass, field

SUPPORTED_SEGMENT_TYPES = ('linear', 'arc')

DEFAULT_SPEED = 0.05      # m/s, for new rows and files saved before speeds existed
DEFAULT_PERIOD = 0.02     # s between dense points (50 Hz)
MIN_LENGTH = 1e-6         # m; shorter than this counts as "doesn't move"


def normalize_speeds(speeds):
    """Return speeds with the first and last set to 0 (start and end at rest)."""
    result = [float(v) for v in speeds]
    if result:
        result[0] = 0.0
        result[-1] = 0.0
    return result


# ----- small vector helpers ---------------------------------------------------

def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _scale(a, k):
    return tuple(x * k for x in a)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    return _scale(a, 1.0 / n) if n > 0 else None


# ----- segment shapes ----------------------------------------------------------

class _Line:

    def __init__(self, start, end):
        self.start, self.end = tuple(start), tuple(end)
        self.length = math.dist(start, end)
        self.exit_dir = _unit(_sub(end, start)) if self.length >= MIN_LENGTH else None

    def point(self, u):
        return tuple(s + u * (e - s) for s, e in zip(self.start, self.end))


class _Arc:
    """Circular arc from start, leaving along unit vector d, ending at end.

    With w = end - start split into a part along d and a part w_perp across
    it, the circle's radius is |w|^2 / (2 |w_perp|), its centre is r along the
    unit vector n = w_perp / |w_perp|, and it sweeps twice the angle between d
    and w. Points are spaced evenly along the arc, so u is a fraction of the
    arc length, the same as for a line.
    """

    def __init__(self, start, end, d, w, w_perp):
        self.start, self.end = tuple(start), tuple(end)
        across = _norm(w_perp)
        self._d = d
        self._n = _scale(w_perp, 1.0 / across)
        self.radius = _dot(w, w) / (2.0 * across)
        self.sweep = 2.0 * math.atan2(across, _dot(w, d))
        self._centre = _add(self.start, _scale(self._n, self.radius))
        self.length = self.radius * self.sweep
        self.exit_dir = _add(_scale(d, math.cos(self.sweep)),
                             _scale(self._n, math.sin(self.sweep)))

    def point(self, u):
        if u >= 1.0:
            return self.end   # exact, so rows land exactly on their points
        a = u * self.sweep
        offset = _add(_scale(self._n, -math.cos(a)), _scale(self._d, math.sin(a)))
        return _add(self._centre, _scale(offset, self.radius))


def build_segments(points, types):
    """Make one shape per gap for one path (camera or focus).

    Returns (segments, notes): notes[i] is '' or a short message about
    segment i (an arc that had to be drawn as a line, or that loops back).
    """
    segments, notes = [], []
    incoming = None   # direction the path is moving in as it reaches the current row
    for i in range(len(points) - 1):
        start, end = points[i], points[i + 1]
        seg_type = types[i] if i < len(types) else 'linear'
        note = ''
        seg = None
        if seg_type == 'arc':
            seg, note = _make_arc(start, end, incoming)
        if seg is None:
            seg = _Line(start, end)
        segments.append(seg)
        notes.append(note)
        # A segment that doesn't move (a hold) keeps the previous direction.
        if seg.exit_dir is not None:
            incoming = seg.exit_dir
    return segments, notes


def _make_arc(start, end, incoming):
    """Return (arc or None, note). None means draw a line instead."""
    w = _sub(end, start)
    if _norm(w) < MIN_LENGTH:
        return None, ''
    if incoming is None:
        return None, 'arc needs a moving segment before it to set its direction; drawn as a line'
    w_perp = _sub(w, _scale(incoming, _dot(w, incoming)))
    if _norm(w_perp) < 1e-9:
        if _dot(w, incoming) > 0:
            return None, ''   # straight ahead: the arc is a line
        return None, 'arc target is straight behind; drawn as a line'
    arc = _Arc(start, end, incoming, w, w_perp)
    note = ''
    if arc.sweep > math.pi + 1e-9:
        note = (f'arc loops back {math.degrees(arc.sweep):.0f}° '
                'because the next row is behind the direction of travel')
    return arc, note


# ----- timing -----------------------------------------------------------------

def _progress(tau, v0, v1, s_curve=False):
    """Fraction of the segment's distance covered at fraction tau of its time.

    Constant acceleration: v(tau) = v0 + (v1 - v0) tau.
    S-curve: v(tau) = v0 + (v1 - v0) (3 tau^2 - 2 tau^3), so dv/dtau = 0 at both ends.
    Both average (v0 + v1) / 2, so at tau = 1 this is exactly 1 either way.
    """
    if s_curve:
        ramp = tau ** 3 - 0.5 * tau ** 4        # integral of the smoothstep
    else:
        ramp = 0.5 * tau * tau
    return 2.0 * (v0 * tau + (v1 - v0) * ramp) / (v0 + v1)


@dataclass
class SegmentTiming:
    duration: float = 0.0    # seconds; 0 when nothing moves
    problem: str = ''        # why it can't be timed; empty when it can
    note: str = ''           # something worth knowing about its shape; it still plays


def _timings(camera_segs, focus_segs, camera_notes, focus_notes, speeds):
    timings = []
    for i, (cam, foc) in enumerate(zip(camera_segs, focus_segs)):
        length = cam.length if cam.length >= MIN_LENGTH else foc.length
        v0, v1 = speeds[i], speeds[i + 1]
        notes = [f'{column}: {n}' for column, n in
                 (('camera', camera_notes[i]), ('focus', focus_notes[i])) if n]
        timing = SegmentTiming(note='; '.join(notes))
        if length < MIN_LENGTH:
            pass
        elif v0 + v1 <= 0.0:
            timing.problem = ('starts and ends at speed 0, so it never gets going: '
                              'give one end a speed, or add a row between')
        else:
            timing.duration = 2.0 * length / (v0 + v1)
        timings.append(timing)
    return timings


def segment_timings(cameras, focuses, camera_types, focus_types, speeds):
    """Time every segment. Returns one SegmentTiming per gap between rows."""
    camera_segs, camera_notes = build_segments(cameras, camera_types)
    focus_segs, focus_notes = build_segments(focuses, focus_types)
    return _timings(camera_segs, focus_segs, camera_notes, focus_notes,
                    normalize_speeds(speeds))


@dataclass
class TimedPath:
    camera: list = field(default_factory=list)    # dense (x, y, z) points
    focus: list = field(default_factory=list)     # same length, paired by index
    times: list = field(default_factory=list)     # seconds from the start, one per point
    # Dense index of each row that the path reaches (rows after a problem are left out).
    row_index: list = field(default_factory=list)
    timings: list = field(default_factory=list)   # SegmentTiming per segment
    problem_segment: int = -1                     # first segment that can't be timed, or -1

    @property
    def duration(self):
        return self.times[-1] if self.times else 0.0

    def locate(self, index):
        """Where dense point `index` is, in table terms.

        Returns ('row', r) when the point is exactly on row r, otherwise
        ('segment', s, fraction) with fraction in (0, 1) of the time between
        row s and row s + 1.
        """
        r = bisect.bisect_right(self.row_index, index) - 1
        if self.row_index[r] == index:
            return ('row', r)
        start, end = self.row_index[r], self.row_index[r + 1]
        return ('segment', r, (index - start) / (end - start))


def sample_timed(cameras, focuses, camera_types, focus_types, speeds,
                 period=DEFAULT_PERIOD, on_unknown_type=None, s_curve=False):
    """Sample both paths on one timeline. See the module docstring."""
    if on_unknown_type:
        for column, types in (('camera', camera_types), ('focus', focus_types)):
            for i, seg_type in enumerate(types):
                if seg_type not in SUPPORTED_SEGMENT_TYPES:
                    on_unknown_type(column, i, seg_type)

    speeds = normalize_speeds(speeds)
    camera_segs, camera_notes = build_segments(cameras, camera_types)
    focus_segs, focus_notes = build_segments(focuses, focus_types)
    timings = _timings(camera_segs, focus_segs, camera_notes, focus_notes, speeds)
    path = TimedPath(timings=timings)
    if not cameras:
        return path

    t = 0.0
    last_row = len(cameras) - 1
    for i, timing in enumerate(timings):
        if timing.problem:
            path.problem_segment = i
            last_row = i
            break
        path.row_index.append(len(path.camera))
        steps = math.ceil(timing.duration / period - 1e-9)
        for k in range(steps):
            tau = k / steps
            u = _progress(tau, speeds[i], speeds[i + 1], s_curve)
            path.camera.append(camera_segs[i].point(u))
            path.focus.append(focus_segs[i].point(u))
            path.times.append(t + tau * timing.duration)
        t += timing.duration

    # The final reachable row, once.
    path.row_index.append(len(path.camera))
    path.camera.append(tuple(cameras[last_row]))
    path.focus.append(tuple(focuses[last_row]))
    path.times.append(t)
    return path
