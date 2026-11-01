import math
import os
import threading
import time
from pathlib import Path

import pytest

from arm_bridge import protocol as p
from arm_bridge.bridge import ArmBridge
from arm_bridge.fake_board import BoardSim, serve
from arm_bridge.planner import ArmConfig, PlanError, plan_run

CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'arm.yaml'


@pytest.fixture
def config():
    return ArmConfig.load(CONFIG)


def _ready_sim():
    sim = BoardSim()
    for line in ['HELLO', 'LIMITS THETA -8000 8000 4000', 'LIMITS Z -60000 0 9000',
                 'LIMITS R -32000 0 9000', 'LIMITS YAW -2000 2000 2000',
                 'LIMITS PITCH -1000 1000 2000', 'ENABLE', 'SETPOS 0 0 0 0 0']:
        assert ' OK ' in sim.handle(line, 0.0)[0], line
    return sim


# ----- protocol --------------------------------------------------------------

def test_reply_round_trip():
    reply = p.parse_line(p.format_ok(7, 12, state='IDLE', pos='1,2,3,4,5'))
    assert (reply.seq, reply.ok, reply.free) == ('7', True, 12)
    assert p.parse_positions(reply.values['pos']) == [1, 2, 3, 4, 5]
    err = p.parse_line(p.format_err(8, p.TOO_FAST, 3, 'THETA needs 9000 steps/s'))
    assert (err.ok, err.code, err.free, err.message) == (False, p.TOO_FAST, 3,
                                                         'THETA needs 9000 steps/s')
    event = p.parse_line(p.format_event(p.FAULT, code='HOME_FAILED'))
    assert event == p.Event('FAULT', {'code': 'HOME_FAILED'})


def test_hand_typed_request_has_no_seq():
    assert p.parse_request('status') == ('-', 'STATUS', [])
    assert p.parse_request('12 SEG 20000 1 2 3 4 5')[:2] == ('12', 'SEG')


# ----- fake board ------------------------------------------------------------

def test_motion_needs_limits_homing_and_enable():
    sim = BoardSim()
    assert 'NO_LIMITS' in sim.handle('1 SEG 20000 0 0 0 0 0', 0)[0]
    sim = _ready_sim()
    sim.handle('DISABLE', 0)
    assert 'NOT_HOMED' in sim.handle('1 SEG 20000 0 0 0 0 0', 0)[0]


def test_segment_checks():
    sim = _ready_sim()
    assert 'OUT_OF_LIMITS' in sim.handle('1 SEG 20000 9000 0 0 0 0', 0)[0]
    assert 'TOO_FAST' in sim.handle('2 SEG 20000 100 0 0 0 0', 0)[0]     # 5000 steps/s
    assert ' OK q=63' in sim.handle('3 SEG 20000 50 0 0 0 0', 0)[0]      # 2500 steps/s


def test_plays_segments_and_reports_done():
    sim = _ready_sim()
    sim.handle('1 SEG 100000 100 0 0 0 0', 0.0)
    sim.handle('2 SEG 100000 200 0 0 0 0', 0.0)
    sim.handle('3 END', 0.0)
    sim.handle('4 GO', 0.0)
    for t in (0.05, 0.1, 0.15):
        sim.handle('PING', t)
        assert sim.update(t) == []
    assert sim.pos[0] == pytest.approx(150)
    assert 'seg=2' in sim.handle('STATUS', 0.15)[0]
    assert sim.update(0.21) == ['! DONE\n']
    assert sim.state == p.IDLE and sim.pos[0] == 200


def test_underrun_ramps_down_and_stays_in_limits():
    sim = _ready_sim()
    sim.handle('1 SEG 100000 300 0 0 0 0', 0.0)   # 3000 steps/s
    sim.handle('2 GO', 0.0)
    sim.update(0.1)                                # queue empty, no END
    assert sim.state == p.STOPPING
    sim.handle('PING', 0.15)
    assert sim.update(0.25) == ['! UNDERRUN\n']
    # Ramped from 3000 steps/s to 0 over 0.1 s: 150 steps further.
    assert sim.pos[0] == pytest.approx(450)


def test_watchdog_stops_a_silent_run():
    sim = _ready_sim()
    sim.handle('1 SEG 1000000 1000 0 0 0 0', 0.0)
    sim.handle('2 GO', 0.0)
    sim.update(0.2)
    assert sim.state == p.RUNNING
    sim.update(0.3)                                # 0.3 s since the last line
    assert sim.state == p.STOPPING
    assert sim.update(0.5) == ['! WATCHDOG\n']


def test_stop_clears_the_queue():
    sim = _ready_sim()
    for i in range(5):
        sim.handle(f'{i + 1} SEG 100000 {50 * (i + 1)} 0 0 0 0', 0.0)
    sim.handle('6 GO', 0.0)
    assert ' OK q=64' in sim.handle('7 STOP', 0.05)[0]
    assert sim.update(0.2) == ['! STOPPED\n']
    assert sim.state == p.IDLE and sim.pos[0] < 100


# ----- planner ---------------------------------------------------------------

def test_plan_moves_to_start_then_follows_path(config):
    times = [0.0, 0.02, 0.04]
    path = [[0.1, 0.4, 0.3, 0.0, 0.0], [0.11, 0.4, 0.3, 0.0, 0.0], [0.12, 0.4, 0.3, 0.0, 0.0]]
    home = [0] * 5
    segments, move, duration = plan_run(config, home, times, path)
    assert duration == pytest.approx(0.04)
    assert sum(d for d, _ in segments) == round((move + 0.04) * 1e6)
    assert segments[-1][1] == config.to_steps(path[-1])
    assert all(p.MIN_SEGMENT_US <= d <= p.MAX_SEGMENT_US for d, _ in segments)
    # The move to the start obeys every joint's speed limit.
    prev = home
    for d, steps in segments:
        for j, a, b in zip(config.joints, prev, steps):
            assert abs(b - a) * 1e6 / d <= j.max_rate()
        prev = steps


def test_plan_refuses_a_path_that_is_too_fast(config):
    times = [0.0, 0.02]
    path = [[0.0, 0.4, 0.3, 0.0, 0.0], [0.1, 0.4, 0.3, 0.0, 0.0]]   # Theta at 5 rad/s
    with pytest.raises(PlanError, match='Theta needs 5.000 rad/s'):
        plan_run(config, [0] * 5, times, path)


def test_plan_splits_long_holds(config):
    start = config.to_steps([0.0, 0.45, 0.39, 0.0, 0.0])
    times = [0.0, 2.5]
    path = [[0.0, 0.45, 0.39, 0.0, 0.0]] * 2
    segments, _, _ = plan_run(config, start, times, path)
    assert all(d <= p.MAX_SEGMENT_US for d, _ in segments)


# ----- bridge + fake board over a pseudo-terminal ---------------------------

@pytest.fixture
def board(tmp_path):
    link = str(tmp_path / 'fake_octopus')
    sim = BoardSim(home_time=0.2)
    thread = threading.Thread(target=serve, args=(link, sim), daemon=True)
    thread.start()
    for _ in range(100):
        if os.path.exists(link):
            break
        time.sleep(0.01)
    return link, sim


def _poll_until(bridge, condition, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        bridge.poll()
        if condition():
            return
        time.sleep(0.02)
    raise AssertionError(f'timed out; last message: {bridge.message!r}')


def test_bridge_homes_and_runs_a_path(config, board):
    link, sim = board
    bridge = ArmBridge(config, link, log=lambda m: None)
    _poll_until(bridge, lambda: bridge.connected and bridge.board)
    assert sim.limits[0] == config.joints[0].step_limits() + (config.joints[0].max_rate(),)

    ok, message = bridge.start_run([0.0], [[0.0] * 5])
    assert not ok and 'Enable' in message
    assert bridge.enable(True)[0]
    assert bridge.home()[0]
    _poll_until(bridge, lambda: bridge.message == 'Homed' and bridge.board.get('homed') == '1')

    # A slow sweep of Theta and Pitch, sampled every 20 ms like camera_path_ik.
    times = [i * 0.02 for i in range(51)]
    path = [[0.4 * t, 0.45, 0.39, 0.0, -0.5 * t] for t in times]
    ok, message = bridge.start_run(times, path)
    assert ok, message
    _poll_until(bridge, lambda: bridge.message == 'Path finished')
    assert bridge.run is None
    assert bridge.positions[0] == pytest.approx(0.4, abs=1e-3)
    assert bridge.positions[4] == pytest.approx(-0.5, abs=3e-3)


def test_bridge_stop_mid_run(config, board):
    link, sim = board
    bridge = ArmBridge(config, link, log=lambda m: None)
    _poll_until(bridge, lambda: bridge.connected and bridge.board)
    bridge.enable(True)
    bridge.assume_homed()
    times = [i * 0.02 for i in range(200)]
    path = [[0.3 * math.sin(t), 0.45, 0.39, 0.0, 0.0] for t in times]
    assert bridge.start_run(times, path)[0]
    _poll_until(bridge, lambda: bridge.board.get('state') == p.RUNNING)
    assert bridge.stop()[0]
    _poll_until(bridge, lambda: bridge.board.get('state') == p.IDLE)
    assert bridge.message == 'Stopped' and bridge.run is None
