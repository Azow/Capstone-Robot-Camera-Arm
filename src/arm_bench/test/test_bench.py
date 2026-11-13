import os
import threading
import time
from pathlib import Path

import pytest

from arm_bridge.fake_board import FAKE_LENS_SPAN, BoardSim, serve
from arm_bridge.planner import ArmConfig
from arm_bridge.protocol import MIN_SEGMENT_US

from arm_bench import profile, results, stall
from arm_bench.bench import MAX_TEST_RATE, TEST_DONE, Bench, bench_config

CONFIG = Path(__file__).resolve().parents[2] / 'arm_bridge' / 'config' / 'arm.yaml'


# ----- profile ---------------------------------------------------------------

def test_trapezoid_and_triangle():
    ta, tc, peak = profile.trapezoid(10000, 1000, 500)     # reaches 1000 steps/s
    assert (ta, peak) == (2.0, 1000)
    assert tc == pytest.approx(8.0)
    ta, tc, peak = profile.trapezoid(-100, 1000, 500)      # too short: a triangle
    assert tc == 0 and peak == pytest.approx((500 * 100) ** 0.5)
    assert profile.trapezoid_position(100.0, -100, 1000, 500) == -100


def _speeds(segments, joint):
    prev = 0
    for dur, steps in segments:
        yield (steps[joint] - prev) * 1e6 / dur
        prev = steps[joint]


def test_move_is_symmetric_and_within_speed():
    b = profile.SegmentBuilder([0] * 6, 2)
    b.move_to(20000, 4000, 8000)
    assert b.segments[-1][1][2] == 20000
    assert all(s[1][k] == 0 for s in b.segments for k in range(6) if k != 2)
    assert all(MIN_SEGMENT_US <= d for d, _ in b.segments)
    speeds = list(_speeds(b.segments, 2))
    assert max(speeds) <= 4000 + 100            # 1 step of rounding per 10 ms segment
    assert speeds == pytest.approx(speeds[::-1], abs=101)   # same ramp up and down
    assert b.duration == pytest.approx(20000 / 4000 + 4000 / 8000, abs=1e-4)


def test_cycle_test_returns_to_lower_end():
    b = profile.cycle_test([0, 0, 0, 500, 0, 0], 3, -3000, 5000, 20000, 3, 0.2, 1000, 4000)
    assert b.segments[-1][1][3] == 0
    ends = [s[1][3] for s in b.segments]
    assert ends.count(-3000) >= 3 and min(ends) == -3000 and max(ends) == 500


def test_tiny_move_is_at_least_one_segment():
    b = profile.SegmentBuilder([0] * 6, 0)
    b.move_to(1, 30000, 1e7)
    assert b.segments == [(MIN_SEGMENT_US, [1, 0, 0, 0, 0, 0])]


# ----- results ---------------------------------------------------------------

def test_results_and_ranges_round_trip(tmp_path):
    path = str(tmp_path / 'r' / 'results.csv')
    results.append_result(path, {'joint': 'YAW', 'result': results.ACCEPTABLE, 'note': 'a, b'})
    results.append_result(path, {'joint': 'Z', 'result': results.SKIPPED})
    rows = results.load_results(path)
    assert [r['joint'] for r in rows] == ['YAW', 'Z'] and rows[0]['note'] == 'a, b'
    ranges = str(tmp_path / 'ranges.yaml')
    results.save_ranges(ranges, {'THETA': -1234})
    assert results.load_ranges(ranges) == {'THETA': -1234}


# ----- against the fake board ------------------------------------------------

def test_bench_config_opens_limits_to_board_cap():
    config = bench_config(ArmConfig.load(CONFIG))
    for j in config.joints:
        assert 39000 < j.max_rate() <= 40000
        lo, hi = j.step_limits()
        assert lo < -1e8 and hi > 1e8


@pytest.fixture
def bench(tmp_path):
    link = str(tmp_path / 'octopus')
    threading.Thread(target=serve, args=(link, BoardSim()), daemon=True).start()
    for _ in range(300):
        if os.path.exists(link):
            break
        time.sleep(0.01)
    b = Bench(ArmConfig.load(CONFIG), link, str(tmp_path / 'ranges.yaml'),
              str(tmp_path / 'results.csv'), log=lambda m: None)
    assert b.connect(link)
    yield b
    b.disconnect()


def _poll_until(bench, condition, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        bench.poll()
        if condition():
            return
        time.sleep(0.01)
    raise AssertionError(f'timed out; last message: {bench.bridge.message!r}')


def test_range_test_and_verdict(bench, tmp_path):
    yaw = 3
    ok, report = bench.enable_all()
    assert ok and bench.enabled, report
    assert not bench.jog(yaw, 10.0, 90.0, 360.0)[0]      # position unknown yet
    assert bench.set_lower_here(yaw)[0] and bench.homed

    assert bench.jog(yaw, -20.0, 90.0, 360.0)[0]
    _poll_until(bench, lambda: bench.bridge.message == 'Yaw moved')
    upper = bench.steps[yaw]
    assert upper == round(-20 * 3.14159265 / 180 * 7053.71)
    assert bench.set_upper_here(yaw)[0]
    assert results.load_ranges(str(tmp_path / 'ranges.yaml')) == {'YAW': upper}

    # 270 deg/s = 33,000 steps/s: near the board's cap.
    assert bench.run_test(yaw, 270.0, 3000.0, 2, 0.05, 90.0, 360.0)[0]
    assert not bench.awaiting_verdict
    _poll_until(bench, lambda: bench.awaiting_verdict)
    assert bench.bridge.message == TEST_DONE and bench.steps[yaw] == 0
    assert bench.record_verdict(results.ACCEPTABLE, 'quiet')
    row, = results.load_results(str(tmp_path / 'results.csv'))
    assert (row['joint'], row['result'], row['cycles'], row['note']) == ('YAW', 'acceptable',
                                                                         '2', 'quiet')
    assert int(row['speed_steps_s']) == round(1.5 * 3.14159265 * 7053.71)


def test_set_lower_after_jogging_keeps_upper_in_place(bench):
    theta = 0
    bench.enable_all()
    bench.set_lower_here(theta)
    bench.jog(theta, 30.0, 90.0, 360.0)
    _poll_until(bench, lambda: bench.bridge.message == 'Theta moved')
    bench.set_upper_here(theta)
    upper = bench.upper(theta)
    bench.jog(theta, -10.0, 90.0, 360.0)
    _poll_until(bench, lambda: bench.bridge.message == 'Theta moved')
    here = bench.steps[theta]
    assert bench.set_lower_here(theta)[0]
    assert bench.upper(theta) == upper - here and bench.steps[theta] == 0


def test_stop_mid_test_can_still_be_marked(bench):
    bench.enable_all()
    bench.set_lower_here(0)
    bench.ranges['THETA'] = 20000
    assert bench.run_test(0, 10.0, 20.0, 5, 0.1, 10.0, 20.0)[0]
    _poll_until(bench, lambda: bench.board.get('state') == 'RUNNING')
    bench.stop()
    _poll_until(bench, lambda: bench.awaiting_verdict)
    assert bench.test['stopped_early']
    assert bench.record_verdict(results.SKIPPED)
    row, = results.load_results(bench.results_path)
    assert row['note'].startswith('stopped early')


def test_too_fast_is_refused(bench):
    bench.enable_all()
    bench.set_lower_here(4)
    bench.ranges['PITCH'] = 1000
    ok, message = bench.run_test(4, 1e5, 1e5, 1, 0.1, 10.0, 10.0)
    assert not ok and str(MAX_TEST_RATE) in message


def test_console_passes_commands_through(bench):
    out = bench.send_raw('status')
    assert out[-1].startswith('OK') and 'state=IDLE' in out[-1]
    assert bench.send_raw('BOGUS')[-1].startswith('ERR BAD_CMD')


# ----- focus stall test -------------------------------------------------------

def test_stall_compare_matches_arm_yaml():
    joint = ArmConfig.load(CONFIG).joints[5]
    predicted = stall.predicted_span(joint)
    c = stall.compare(joint, predicted)
    assert c['error_pct'] == pytest.approx(0.0)
    assert c['implied_lens_teeth'] == pytest.approx(108.0, abs=0.1)   # arm.yaml's estimate
    assert stall.parse_sample('SG t=125 steps=60 sg=231') == (0.125, 60, 231)
    assert stall.parse_sample('7 OK q=64') is None


def _seek(bench, steps, sgthrs=30, current=180, rate=4000):
    ok, message = bench.start_stall_seek(steps, rate, sgthrs, current, {'sgthrs': sgthrs})
    assert ok, message
    _poll_until(bench, lambda: bench.seek is None)
    return bench.seek_result


def test_stall_seek_finds_the_fake_lens_span(bench, tmp_path):
    focus = 5
    bench.enable_all()
    bench.set_zero_here(focus)          # the fake ring starts on its close-focus stop
    row = _seek(bench, -6000)
    assert row['result'] == stall.STALL_RESULT
    assert row['span_steps'] == FAKE_LENS_SPAN
    assert float(row['error_pct']) < 0      # the fake lens is a little short
    assert len(bench.seek_samples) > 3
    assert bench.steps[focus] == -FAKE_LENS_SPAN   # the board kept counting
    assert stall.load(bench.stall_path)[-1]['span_steps'] == str(FAKE_LENS_SPAN)
    # Back the other way, from the infinity stop: same span, measured from zero.
    assert bench.set_zero_here(focus)[0]
    assert _seek(bench, 6000)['span_steps'] == FAKE_LENS_SPAN


def test_stall_seek_limit_and_false_trip(bench):
    bench.enable_all()
    bench.set_zero_here(5)
    assert _seek(bench, -500)['result'] == stall.LIMIT_RESULT    # stopped short of the stop
    row = _seek(bench, -4000, sgthrs=255)                      # threshold above free running
    assert row['result'] == stall.STALL_RESULT and row['span_steps'] < 1000


def test_emergency_stop_aborts_seek_and_disables(bench):
    bench.enable_all()
    bench.set_zero_here(5)
    assert bench.start_stall_seek(-6000, 500, 30, 180, {})[0]
    _poll_until(bench, lambda: len(bench.seek_samples) > 2)
    bench.emergency_stop()
    _poll_until(bench, lambda: bench.seek is None and not bench.enabled)
    assert bench.seek_result['result'] == stall.ABORTED_RESULT
    assert not stall.load(bench.stall_path)                     # aborted runs aren't saved


# ----- single-joint power (bench firmware) -----------------------------------

def test_fake_board_powers_single_joints():
    sim = BoardSim()
    for line in ['HELLO'] + [f'LIMITS {j} -100000 100000 20000'
                             for j in ('THETA', 'Z', 'R', 'YAW', 'PITCH', 'FOCUS')] + [
            'SETPOS 0 0 0 0 0 0', 'ENABLE YAW']:
        assert ' OK ' in sim.handle(line, 0.0)[0], line
    status = sim.handle('STATUS', 0.0)[0]
    assert 'enabled=1' in status and 'power=0,0,0,1,0,0' in status and 'homed=1' in status
    assert 'DISABLED' in sim.handle('1 SEG 20000 10 0 0 0 0 0', 0.0)[0]   # THETA is off
    assert ' OK ' in sim.handle('2 SEG 20000 0 0 0 10 0 0', 0.0)[0]       # YAW is on
    assert 'BAD_ARGS' in sim.handle('ENABLE ELBOW', 0.0)[0]
    sim.handle('DISABLE YAW', 0.0)
    status = sim.handle('STATUS', 0.0)[0]
    assert 'enabled=0' in status and 'homed=1' in status   # single-joint DISABLE keeps homed
    sim.handle('ENABLE', 0.0)
    sim.handle('DISABLE', 0.0)
    assert 'homed=0' in sim.handle('STATUS', 0.0)[0]       # plain DISABLE still clears it


def test_bench_powers_one_joint_and_tracks_lost_positions(bench):
    theta, yaw = 0, 3
    assert bench.per_joint_power
    assert bench.set_power({theta, yaw})[0]
    bench.set_lower_here(theta)
    bench.set_lower_here(yaw)
    assert bench.position_known(theta) and bench.position_known(yaw)

    ok, message = bench.set_power({yaw})
    assert ok and message.startswith('Powered: YAW')
    assert bench.power == [False, False, False, True, False, False]
    assert bench.position_known(yaw) and not bench.position_known(theta)
    ok, message = bench.jog(theta, 5.0, 90.0, 360.0)
    assert not ok and 'not powered' in message
    assert bench.jog(yaw, 5.0, 90.0, 360.0)[0]
    _poll_until(bench, lambda: bench.bridge.message == 'Yaw moved')

    # Back on: still unknown (it was limp) until it's zeroed again.
    bench.set_power({theta})
    ok, message = bench.jog(theta, 5.0, 90.0, 360.0)
    assert not ok and 'unknown' in message
    assert bench.set_lower_here(theta)[0] and bench.position_known(theta)
    assert bench.jog(theta, 5.0, 90.0, 360.0)[0]
