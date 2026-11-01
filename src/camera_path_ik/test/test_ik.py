import math

import numpy as np
import pytest

from camera_path_ik import ik_numeric
from camera_path_ik.arm_model import ArmModel, PITCH, R, THETA, YAW
from camera_path_ik.ik import IkSolver
from camera_path_ik.kinematics import Chain

from arms import FULL_PITCH_ARM, REAL_URDF, sample_limits, TEST_ARM, THROUGH_AXIS_ARM

needs_real_urdf = pytest.mark.skipif(not REAL_URDF.exists(),
                                     reason='arm_description_v2 URDF not found')


def _solver(urdf_text):
    return IkSolver(ArmModel.from_urdf(urdf_text))


def _assert_hits(solver, result, camera, focus):
    assert result.reached, result.reason
    position, look = solver.model.camera_pose(result.q)
    np.testing.assert_allclose(position, camera, atol=1e-6)
    aim = (np.asarray(focus) - camera) / np.linalg.norm(np.asarray(focus) - camera)
    assert math.acos(min(1.0, look @ aim)) < 1e-5
    assert np.all(result.q >= solver.model.lower) and np.all(result.q <= solver.model.upper)


def _round_trip(urdf_text, count=300):
    solver = _solver(urdf_text)
    m = solver.model
    rng = np.random.default_rng(0)
    for _ in range(count):
        q = rng.uniform(*sample_limits(m.lower, m.upper))
        camera, look = m.camera_pose(q)
        focus = camera + look * rng.uniform(0.05, 1.0)
        _assert_hits(solver, solver.solve(camera, focus), camera, focus)


# ----- reaching points the arm can reach -------------------------------------

def test_round_trip_on_test_arm():
    _round_trip(TEST_ARM)


def test_round_trip_through_axis_arm():
    _round_trip(THROUGH_AXIS_ARM)


@needs_real_urdf
def test_round_trip_on_real_urdf():
    _round_trip(REAL_URDF.read_text())


@needs_real_urdf
def test_path_moves_joints_smoothly():
    solver = _solver(REAL_URDF.read_text())
    start, end = np.array([-0.2, 0.05, 0.1]), np.array([0.05, -0.2, 0.05])
    cameras = [start + u * (end - start) for u in np.linspace(0, 1, 101)]
    focuses = [np.array([0.3, 0.3, 0.0])] * len(cameras)
    results = solver.solve_path(cameras, focuses)
    assert all(r.reached for r in results), results[-1].reason
    steps = np.abs(np.diff([r.q for r in results], axis=0))
    assert steps.max() < 0.1   # no jumps: every joint moves less than 0.1 per step


# ----- points where a joint stops mattering ----------------------------------

def test_theta_is_held_when_wrist_is_on_the_axis():
    solver = _solver(THROUGH_AXIS_ARM)
    # Wrist on the axis (R = 0.05), camera 0.04 m in front of it looking along +y.
    camera, focus = np.array([0.0, 0.04, 0.18]), np.array([0.0, 1.0, 0.18])
    previous = np.array([0.7, 0.0, 0.05, 0.0, 0.0])
    result = solver.solve(camera, focus, previous)
    _assert_hits(solver, result, camera, focus)
    assert result.q[THETA] == pytest.approx(0.7)
    assert result.q[R] == pytest.approx(0.05)
    assert any('Theta' in h for h in result.held)


def test_slide_through_the_axis_instead_of_flipping_theta():
    # A wrist 0.03 m from the axis at angle a can be reached two ways: R = 0.08
    # with Theta = a, or R = 0.02 (wrist on the far side) with Theta = a + 180 deg
    # and Yaw turned 180 deg to keep the same aim. The one closest to the previous
    # pose wins. a = -0.5 and a sideways aim keep both inside the +-3.14 limits
    # (Yaw = +90 deg or -90 deg); aiming straight out would need Yaw = 180 deg.
    solver = _solver(THROUGH_AXIS_ARM)
    a = -0.5
    out = np.array([math.cos(a), math.sin(a), 0.0])
    side = np.array([-math.sin(a), math.cos(a), 0.0])
    camera = 0.03 * out + 0.04 * side + np.array([0.0, 0.0, 0.18])   # wrist + head offset
    focus = camera + side
    near_a = solver.solve(camera, focus, np.array([-0.4, 0.0, 0.1, 0.0, 0.0]))
    near_flip = solver.solve(camera, focus, np.array([2.5, 0.0, 0.0, 0.0, 0.0]))
    _assert_hits(solver, near_a, camera, focus)
    _assert_hits(solver, near_flip, camera, focus)
    assert near_a.q[R] == pytest.approx(0.08)
    assert near_a.q[THETA] == pytest.approx(a)
    assert near_flip.q[R] == pytest.approx(0.02)
    assert near_flip.q[THETA] == pytest.approx(a + math.pi)


@pytest.mark.parametrize('direction', [+1, -1])
def test_yaw_is_held_when_looking_straight_up_or_down(direction):
    solver = _solver(FULL_PITCH_ARM)
    m = solver.model
    # On this arm positive Pitch tilts the camera down.
    target = np.array([0.3, 0.0, 0.1, 0.4, -direction * math.pi / 2])
    camera, _ = m.camera_pose(target)
    focus = camera + direction * np.array([0.0, 0.0, 0.3])
    previous = np.array([0.3, 0.0, 0.1, 0.4, 0.0])
    result = solver.solve(camera, focus, previous)
    _assert_hits(solver, result, camera, focus)
    assert any('Yaw' in h for h in result.held)
    np.testing.assert_allclose(result.q, target, atol=1e-9)


def test_camera_on_its_focus_point_keeps_the_old_aim():
    solver = _solver(TEST_ARM)
    previous = np.array([0.2, 0.0, 0.1, -0.3, 0.4])
    camera, _ = solver.model.camera_pose(previous)
    result = solver.solve(camera, camera, previous)
    assert result.reached
    assert result.q[PITCH] == pytest.approx(0.4)
    assert result.q[THETA] + result.q[YAW] == pytest.approx(previous[THETA] + previous[YAW])


# ----- points the arm cannot reach -------------------------------------------

@pytest.mark.parametrize('camera, focus, reason', [
    ((0.15, 0.02, 0.5), (1.0, 0.02, 0.5), 'camera too high'),
    ((0.15, 0.02, -0.2), (1.0, 0.02, -0.2), 'camera too low'),
    ((0.5, 0.02, 0.18), (1.0, 0.02, 0.18), 'camera out of reach'),
    # Wrist would be 0.04 m from the axis, but R can only bring it to 0.054 m.
    ((0.0, 0.0, 0.18), (1.0, 0.0, 0.18), 'camera out of reach'),
    # Wrist would be 0.01 m from the axis, inside the 0.02 m side offset.
    ((0.04, 0.01, 0.18), (1.0, 0.01, 0.18), 'camera too close to the Theta axis'),
    # Looking straight up needs Pitch -90 deg; the limit is -1.5 rad (-85.9 deg).
    ((0.15, 0.02, 0.18), (0.15, 0.02, 1.0), 'aim needs Pitch'),
])
def test_unreachable_points_say_why(camera, focus, reason):
    result = _solver(TEST_ARM).solve(camera, focus)
    assert not result.reached
    assert reason in result.reason


def test_every_problem_is_listed():
    result = _solver(TEST_ARM).solve((0.5, 0.02, 0.5), (1.0, 0.02, 0.5))
    assert 'too high' in result.reason and 'out of reach' in result.reason


def test_path_stops_at_first_unreachable_point():
    solver = _solver(TEST_ARM)
    cameras = [(0.15, 0.02, 0.18), (0.16, 0.02, 0.18), (0.5, 0.02, 0.18), (0.15, 0.02, 0.18)]
    focuses = [(1.0, 0.02, 0.18)] * 4
    results = solver.solve_path(cameras, focuses)
    assert [r.reached for r in results] == [True, True, False]


@needs_real_urdf
def test_paths_never_leave_the_joint_limits():
    # Random waypoints, many out of reach: whatever the solver hands back to play
    # must be inside every URDF limit, never clamped past one or wrapped around.
    solver = _solver(REAL_URDF.read_text())
    m = solver.model
    rng = np.random.default_rng(2)
    for _ in range(200):
        cameras = rng.uniform([-0.5, -0.5, -0.15], [0.5, 0.5, 0.7], size=(8, 3))
        focuses = rng.uniform([-1.0, -1.0, -0.5], [1.0, 1.0, 1.0], size=(8, 3))
        for result in solver.solve_path(cameras, focuses):
            if result.reached:
                assert np.all(result.q >= m.lower) and np.all(result.q <= m.upper)


# ----- cross-check against the numerical solver ------------------------------

@needs_real_urdf
def test_numeric_solver_never_reaches_a_point_closed_form_rejects():
    urdf = REAL_URDF.read_text()
    chain = Chain.from_urdf(urdf)
    solver = _solver(urdf)
    rng = np.random.default_rng(3)
    agreed_reachable = 0
    for _ in range(12):
        # Random targets in a box a bit larger than the workspace, so some are out of reach.
        camera = rng.uniform([-0.3, -0.3, -0.05], [0.3, 0.3, 0.22])
        focus = camera + rng.normal(size=3) * 0.3
        closed = solver.solve(camera, focus)
        _, numeric_reached = ik_numeric.solve(chain, camera, focus)
        if numeric_reached:
            assert closed.reached, f'numeric reached {camera} -> {focus}, closed form said: ' \
                                   f'{closed.reason}'
            agreed_reachable += 1
        if closed.reached:
            _assert_hits(solver, closed, camera, focus)
    assert agreed_reachable > 0   # the check is only meaningful if some points were reachable


def test_theta_stops_at_its_limit_instead_of_spinning_a_full_turn():
    # Circle the camera around the base axis past Theta = +3.14. Continuing
    # smoothly needs Theta > 3.14; the in-limits value (-3.14) is a full turn
    # back, which a real arm would have to spin through. That must not happen.
    solver = _solver(TEST_ARM)
    m = solver.model
    poses = [np.array([t, 0.0, 0.1, 0.0, 0.0]) for t in np.linspace(2.9, 3.4, 26)]
    cameras, focuses = [], []
    for q in poses:
        camera, look = m.camera_pose(q)
        cameras.append(camera)
        focuses.append(camera + 0.3 * look)
    results = solver.solve_path(cameras, focuses)
    assert not results[-1].reached
    assert 'Theta would pass its limit' in results[-1].reason
    reached = np.array([r.q for r in results[:-1]])
    assert reached[:, THETA].max() <= m.upper[THETA]
    assert np.abs(np.diff(reached, axis=0)).max() < 0.1


def test_first_point_may_use_any_turn_within_limits():
    # With no previous point there is nothing to continue from, so a target
    # whose Theta is -170 deg is fine even though "near 0" would be +190 deg.
    solver = _solver(TEST_ARM)
    q = np.array([math.radians(-170), 0.0, 0.1, 0.3, 0.0])
    camera, look = solver.model.camera_pose(q)
    result = solver.solve(camera, camera + 0.3 * look)
    _assert_hits(solver, result, camera, camera + 0.3 * look)


def test_theta_keeps_turning_when_its_limits_are_removed():
    # Same circle as the limit test above, with free_spin's remove_limits():
    # Theta now continues smoothly past 3.14 instead of stopping.
    model = ArmModel.from_urdf(TEST_ARM)
    model.remove_limits([THETA, YAW])
    solver = IkSolver(model)
    poses = [np.array([t, 0.0, 0.1, 0.0, 0.0]) for t in np.linspace(2.9, 3.4, 26)]
    cameras, focuses = [], []
    for q in poses:
        camera, look = model.camera_pose(q)
        cameras.append(camera)
        focuses.append(camera + 0.3 * look)
    results = solver.solve_path(cameras, focuses)
    assert all(r.reached for r in results), results[-1].reason
    assert results[-1].q[THETA] == pytest.approx(3.4)
    # The chain read from the URDF keeps its limits.
    assert Chain.from_urdf(TEST_ARM).upper[THETA] == pytest.approx(3.14)
