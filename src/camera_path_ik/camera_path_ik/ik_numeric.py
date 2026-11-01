"""Numerical IK, used by the tests to cross-check the closed-form solver in ik.py.

Pure Python plus numpy/scipy, no ROS imports.

It knows nothing about the arm's layout: it only calls the URDF forward
kinematics and searches for joint positions that make the camera land on the
target and look at the focus. That makes it a fair, independent check: if
ik.py says a point is unreachable, this should not find a solution either,
and when both find one, both must pass the same check.

Each target is a bounded least-squares problem (joints stay inside their
URDF limits) with the error

    [camera position - target,  AIM_WEIGHT * (camera look direction - unit vector to focus)]

It is much slower than ik.py and can miss solutions, so it is not used at runtime.
"""

import math

import numpy as np
from scipy.optimize import least_squares

from camera_path_ik.arm_model import THETA, YAW

# Meters of position error that count the same as 1 rad of aim error.
AIM_WEIGHT = 0.1

POSITION_TOLERANCE = 1e-4                 # meters
AIM_TOLERANCE = math.radians(0.05)        # radians

# Starting guesses: a grid over the two vertical rotations, other joints mid-range.
SEED_ANGLES = np.linspace(-3.0, 3.0, 7)


def errors(chain, q, camera, focus):
    """(position error in meters, aim error in radians) for pose q."""
    pose = chain.fk(q)
    position_error = float(np.linalg.norm(pose[:3, 3] - camera))
    look = focus - pose[:3, 3]
    distance = np.linalg.norm(look)
    if distance < 1e-9:
        return position_error, 0.0
    cos_angle = np.clip(pose[:3, 2] @ (look / distance), -1.0, 1.0)
    return position_error, float(math.acos(cos_angle))


def _residual(q, chain, camera, aim):
    pose = chain.fk(q)
    return np.concatenate([pose[:3, 3] - camera, AIM_WEIGHT * (pose[:3, 2] - aim)])


def _seeds(chain):
    # Continuous joints have infinite limits; start them at 0.
    finite = np.isfinite(chain.lower) & np.isfinite(chain.upper)
    middle = np.zeros(len(chain.lower))
    middle[finite] = (chain.lower[finite] + chain.upper[finite]) / 2
    for theta in SEED_ANGLES:
        for yaw in SEED_ANGLES:
            seed = middle.copy()
            seed[THETA], seed[YAW] = theta, yaw
            yield seed


def solve(chain, camera, focus):
    """Best joint positions found, and whether they reach the target.

    Returns (q, reached).
    """
    camera = np.asarray(camera, dtype=float)
    focus = np.asarray(focus, dtype=float)
    aim = (focus - camera) / np.linalg.norm(focus - camera)
    margin = 1e-6   # least_squares needs the seed strictly inside the bounds
    best_q, best_cost = None, math.inf
    for seed in _seeds(chain):
        seed = np.clip(seed, chain.lower + margin, chain.upper - margin)
        fit = least_squares(_residual, seed, bounds=(chain.lower, chain.upper),
                            args=(chain, camera, aim), xtol=1e-12, ftol=1e-12, gtol=1e-12)
        if fit.cost < best_cost:
            best_q, best_cost = fit.x, fit.cost
        position_error, aim_error = errors(chain, best_q, camera, focus)
        if position_error <= POSITION_TOLERANCE and aim_error <= AIM_TOLERANCE:
            return best_q, True
    return best_q, False
