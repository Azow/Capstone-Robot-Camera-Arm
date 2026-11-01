import math

import pytest

import random

from camera_path_editor.sampling import (
    build_segments, normalize_speeds, sample_timed, segment_timings,
)

TEST_A_CAMERA = [(0.6, -0.4, 0.3), (0.6, 0.0, 0.5), (0.6, 0.4, 0.3)]
TEST_A_FOCUS = [(0.0, 0.0, 0.2)] * 3
LINEAR2 = ['linear'] * 2

# A straight 0.3 m line in x, in three 0.1 m pieces: speed up, cruise, slow down.
LINE = [(0.0, 0, 0), (0.1, 0, 0), (0.2, 0, 0), (0.3, 0, 0)]
LINE_FOCUS = [(0.0, 1, 0), (0.1, 1, 0), (0.2, 1, 0), (0.3, 1, 0)]
LINE_SPEEDS = [0.0, 0.1, 0.1, 0.0]
LINEAR3 = ['linear'] * 3


def _speeds_along(path):
    """Speed between consecutive dense points, from the time stamps."""
    return [math.dist(a, b) / (tb - ta)
            for a, b, ta, tb in zip(path.camera, path.camera[1:], path.times, path.times[1:])]


def test_ends_are_forced_to_rest():
    assert normalize_speeds([0.3, 0.2, 0.4]) == [0.0, 0.2, 0.0]
    assert normalize_speeds([0.3]) == [0.0]
    assert normalize_speeds([]) == []


def test_duration_is_length_over_average_speed():
    timings = segment_timings(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS)
    # 0.1 m at an average of 0.05, 0.1 and 0.05 m/s.
    assert [t.duration for t in timings] == pytest.approx([2.0, 1.0, 2.0])
    assert all(not t.problem for t in timings)


def test_time_stamps_are_evenly_spaced_and_hit_every_row():
    path = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.02)
    assert path.duration == pytest.approx(5.0)
    steps = [b - a for a, b in zip(path.times, path.times[1:])]
    assert steps == pytest.approx([0.02] * len(steps))
    for row, index in enumerate(path.row_index):
        assert path.camera[index] == pytest.approx(LINE[row])
        assert path.focus[index] == pytest.approx(LINE_FOCUS[row])
    assert path.times[path.row_index[1]] == pytest.approx(2.0)
    assert path.times[path.row_index[2]] == pytest.approx(3.0)


def test_speed_ramps_up_cruises_and_ramps_down():
    path = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.02)
    speeds = _speeds_along(path)
    a, b = path.row_index[1], path.row_index[2]
    # Ease in: strictly faster each step, from ~0 up to ~0.1 m/s.
    assert all(s1 < s2 for s1, s2 in zip(speeds[:a], speeds[1:a]))
    assert speeds[0] < 0.01
    # Cruise: constant.
    assert speeds[a:b] == pytest.approx([0.1] * (b - a))
    # Ease out: strictly slower each step.
    assert all(s1 > s2 for s1, s2 in zip(speeds[b:], speeds[b + 1:]))
    assert speeds[-1] < 0.01


def test_speed_is_continuous_across_rows():
    path = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.001)
    speeds = _speeds_along(path)
    for index in path.row_index[1:-1]:
        assert speeds[index] == pytest.approx(speeds[index - 1], abs=1e-3)


def test_focus_shares_the_camera_timeline():
    # The camera moves 0.1 m, the focus 0.4 m: both get to the end together.
    cams = [(0, 0, 0), (0.1, 0, 0)]
    focs = [(0, 1, 0), (0.4, 1, 0)]
    path = sample_timed(cams, focs, ['linear'], ['linear'], [0, 0], period=0.02)
    # Both ends are forced to 0, so that's a problem; give it a middle row instead.
    assert path.problem_segment == 0
    cams = [(0, 0, 0), (0.05, 0, 0), (0.1, 0, 0)]
    focs = [(0, 1, 0), (0.2, 1, 0), (0.4, 1, 0)]
    path = sample_timed(cams, focs, LINEAR2, LINEAR2, [0, 0.1, 0], period=0.02)
    for cam, foc in zip(path.camera, path.focus):
        assert foc[0] == pytest.approx(4 * cam[0])


def test_pan_only_segment_is_timed_by_the_focus_path():
    cams = [(0, 0, 0), (0, 0, 0), (0, 0, 0)]
    focs = [(0, 1, 0), (0.2, 1, 0), (0.4, 1, 0)]
    timings = segment_timings(cams, focs, LINEAR2, LINEAR2, [0, 0.1, 0])
    assert [t.duration for t in timings] == pytest.approx([4.0, 4.0])


def test_segment_that_does_not_move_takes_no_time():
    cams = [(0, 0, 0), (0.1, 0, 0), (0.1, 0, 0), (0.2, 0, 0)]
    focs = [(0, 1, 0)] * 4
    path = sample_timed(cams, focs, LINEAR3, LINEAR3, [0, 0.1, 0.1, 0])
    assert [t.duration for t in path.timings] == pytest.approx([2.0, 0.0, 2.0])
    assert path.row_index[1] == path.row_index[2]
    assert path.locate(path.row_index[1]) == ('row', 2)


def test_stop_to_stop_segment_stops_the_path_there():
    cams = [(0, 0, 0), (0.1, 0, 0), (0.2, 0, 0), (0.3, 0, 0)]
    path = sample_timed(cams, cams, LINEAR3, LINEAR3, [0, 0.1, 0, 0])
    assert path.problem_segment == 2
    assert path.timings[2].problem
    assert len(path.row_index) == 3          # rows 1-3 are reached, row 4 is not
    assert path.camera[-1] == (0.2, 0, 0)


def test_locate_rows_and_segments():
    path = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.02)
    assert path.locate(0) == ('row', 0)
    assert path.locate(len(path.camera) - 1) == ('row', 3)
    kind, segment, fraction = path.locate(path.row_index[1] + 25)
    assert (kind, segment) == ('segment', 1)
    assert fraction == pytest.approx(0.5)


def test_period_sets_point_spacing():
    coarse = sample_timed(TEST_A_CAMERA, TEST_A_FOCUS, LINEAR2, LINEAR2, [0, 0.1, 0], 0.1)
    fine = sample_timed(TEST_A_CAMERA, TEST_A_FOCUS, LINEAR2, LINEAR2, [0, 0.1, 0], 0.01)
    assert coarse.duration == pytest.approx(fine.duration)
    assert len(fine.camera) > 9 * len(coarse.camera)


def test_zero_and_one_rows():
    empty = sample_timed([], [], [], [], [])
    assert empty.camera == [] and empty.times == []
    one = sample_timed([(1.0, 2.0, 3.0)], [(0, 0, 0)], [], [], [0.2])
    assert one.camera == [(1.0, 2.0, 3.0)] and one.times == [0.0]


def test_unknown_type_falls_back_to_linear_and_warns():
    warnings = []
    path = sample_timed([(0, 0, 0), (0.5, 0, 0), (1, 0, 0)], [(0, 1, 0)] * 3,
                        ['spline', 'linear'], LINEAR2, [0, 0.5, 0], 0.5,
                        lambda c, i, t: warnings.append((c, i, t)))
    assert warnings == [('camera', 0, 'spline')]
    assert path.camera[path.row_index[1]] == (0.5, 0, 0)


# ----- tangent arcs -----------------------------------------------------------

def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def _direction_at(seg, u, h=1e-6):
    a, b = seg.point(u - h if u >= h else u), seg.point(min(u + h, 1.0))
    return _unit(tuple(y - x for x, y in zip(a, b)))


def test_quarter_circle_after_a_line():
    # Along +x to (0.1, 0, 0), then an arc to (0.2, 0.1, 0): a quarter circle
    # of radius 0.1 around (0.1, 0.1, 0), leaving along +y.
    pts = [(0, 0, 0), (0.1, 0, 0), (0.2, 0.1, 0)]
    (line, arc), notes = build_segments(pts, ['linear', 'arc'])
    assert notes == ['', '']
    assert arc.radius == pytest.approx(0.1)
    assert arc.sweep == pytest.approx(math.pi / 2)
    assert arc.length == pytest.approx(0.1 * math.pi / 2)
    assert arc.exit_dir == pytest.approx((0, 1, 0), abs=1e-12)
    for k in range(11):
        assert math.dist(arc.point(k / 10), (0.1, 0.1, 0)) == pytest.approx(0.1)
    assert arc.point(0) == pytest.approx(pts[1])
    assert arc.point(1) == pts[2]


def test_arc_is_evenly_spaced_along_its_length():
    (_, arc), _ = build_segments([(0, 0, 0), (0.1, 0, 0), (0.2, 0.1, 0)], ['linear', 'arc'])
    chords = [math.dist(arc.point(k / 20), arc.point((k + 1) / 20)) for k in range(20)]
    assert chords == pytest.approx([chords[0]] * 20)


def test_arcs_in_3d_start_tangent_and_end_on_the_row():
    rng = random.Random(1)
    for _ in range(50):
        pts = [tuple(rng.uniform(-0.5, 0.5) for _ in range(3)) for _ in range(4)]
        segs, notes = build_segments(pts, ['linear', 'arc', 'arc'])
        for i in (1, 2):
            if notes[i]:   # loop-back arcs are still arcs; only skip fallbacks
                assert 'drawn as a line' not in notes[i]
            # Leaves along the previous segment's exit direction, ends on the row.
            assert _direction_at(segs[i], 0.0) == pytest.approx(segs[i - 1].exit_dir, abs=1e-4)
            assert segs[i].point(1.0) == pts[i + 1]
            assert _direction_at(segs[i], 1.0) == pytest.approx(segs[i].exit_dir, abs=1e-4)


def test_arc_needs_a_moving_segment_before_it():
    segs, notes = build_segments([(0, 0, 0), (0.1, 0.1, 0)], ['arc'])
    assert 'drawn as a line' in notes[0]
    assert segs[0].point(0.5) == pytest.approx((0.05, 0.05, 0))


def test_arc_after_a_hold_uses_the_last_moving_direction():
    pts = [(0, 0, 0), (0.1, 0, 0), (0.1, 0, 0), (0.2, 0.1, 0)]
    segs, notes = build_segments(pts, ['linear', 'linear', 'arc'])
    assert notes[2] == ''
    assert segs[2].radius == pytest.approx(0.1)


def test_arc_straight_ahead_is_a_line_and_straight_behind_is_a_note():
    pts = [(0, 0, 0), (0.1, 0, 0), (0.3, 0, 0)]
    segs, notes = build_segments(pts, ['linear', 'arc'])
    assert notes[1] == '' and segs[1].length == pytest.approx(0.2)
    pts = [(0, 0, 0), (0.1, 0, 0), (-0.1, 0, 0)]
    _, notes = build_segments(pts, ['linear', 'arc'])
    assert 'behind' in notes[1]


def test_arc_that_loops_back_gets_a_note():
    # Target behind and to the side: the arc sweeps more than half a circle.
    _, notes = build_segments([(0, 0, 0), (0.1, 0, 0), (0.05, 0.1, 0)], ['linear', 'arc'])
    assert 'loops back' in notes[1]


def test_arc_timing_uses_the_arc_length():
    pts = [(0, 0, 0), (0.1, 0, 0), (0.2, 0.1, 0)]
    timings = segment_timings(pts, pts, ['linear', 'arc'], ['linear', 'arc'], [0, 0.1, 0])
    assert timings[1].duration == pytest.approx(2 * (0.1 * math.pi / 2) / 0.1)


def test_arc_then_arc_has_no_corner_in_the_dense_path():
    # An S-bend of two arcs: the direction between dense points never jumps.
    pts = [(0, 0, 0), (0.1, 0, 0), (0.2, 0.1, 0), (0.3, 0.2, 0.05), (0.4, 0.2, 0.1)]
    types = ['linear', 'arc', 'arc', 'arc']
    path = sample_timed(pts, pts, types, types, [0, 0.1, 0.1, 0.1, 0], period=0.002)
    dirs = [_unit(tuple(b - a for a, b in zip(p, q)))
            for p, q in zip(path.camera, path.camera[1:])]
    worst = max(math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))))
                for a, b in zip(dirs, dirs[1:]))
    assert worst < 3.0


# ----- S-curve -----------------------------------------------------------------

def _accels(path):
    """Acceleration between consecutive speed samples (central differences)."""
    v = _speeds_along(path)
    mid = [(a + b) / 2 for a, b in zip(path.times, path.times[1:])]
    return [(v2 - v1) / (t2 - t1) for v1, v2, t1, t2 in zip(v, v[1:], mid, mid[1:])]


def test_s_curve_keeps_the_same_timing_and_rows():
    plain = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.02)
    smooth = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.02,
                          s_curve=True)
    assert smooth.times == plain.times
    assert smooth.row_index == plain.row_index
    for index in smooth.row_index:
        assert smooth.camera[index] == pytest.approx(plain.camera[index])


def test_s_curve_has_no_acceleration_jumps_at_rows():
    plain = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.002)
    smooth = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.002,
                          s_curve=True)

    def jumps(path):
        # Acceleration a few samples after each middle row minus a few before it.
        acc = _accels(path)
        return [abs(acc[r + 3] - acc[r - 4]) for r in path.row_index[1:-1]]

    # Constant acceleration jumps by 0.05 m/s^2 where the 2 s ease in (0 -> 0.1 m/s)
    # meets the cruise, and again into the ease out. The S-curve doesn't.
    assert jumps(plain) == pytest.approx([0.05, 0.05], rel=0.05)
    assert max(jumps(smooth)) < 0.002
    # And acceleration is ~0 at every row.
    accels = _accels(smooth)
    for index in smooth.row_index[1:-1]:
        assert abs(accels[index - 1]) < 1e-3


def test_s_curve_peak_acceleration_is_one_and_a_half_times():
    plain = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.002)
    smooth = sample_timed(LINE, LINE_FOCUS, LINEAR3, LINEAR3, LINE_SPEEDS, period=0.002,
                          s_curve=True)
    assert max(_accels(smooth)) == pytest.approx(1.5 * max(_accels(plain)), rel=0.02)
