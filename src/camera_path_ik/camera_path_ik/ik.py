"""Closed-form IK: put the camera on a point, looking at another point.

Pure Python plus numpy, no ROS imports, so it can be unit tested on its own.

Works on the numbers measured by arm_model.ArmModel, so it fits any arm with
the Theta -> Z -> R -> Yaw -> Pitch layout. The derivation is in the README;
the step numbers in the comments below match it.

Roll about the viewing axis is not controlled: the arm has no joint for it.

When a joint stops mattering (the wrist is on the Theta axis, or the camera
looks straight up or down), it is held at its previous value instead of
jumping to an arbitrary one.
"""

import math

import numpy as np

from camera_path_ik.arm_model import horizontal, PITCH, R, THETA, YAW, Z
from camera_path_ik.kinematics import axis_angle_matrix

# The self-check after solving must be at least this close. The closed form
# is exact up to how far the URDF's axes are from parallel (about 1e-6 m on
# the CAD export, which writes pi/2 as 1.5708).
POSITION_TOLERANCE = 1e-4                 # meters
AIM_TOLERANCE = math.radians(0.05)        # radians

# Below this, a direction or radius counts as zero and its angle is undefined.
SINGULAR = 1e-6

# Slack when comparing against joint limits, so a value computed as
# 0.2000000001 for a limit of 0.2 still counts as inside.
LIMIT_SLACK = 1e-9

# For picking between solutions: 0.1 m of slide counts the same as 1 rad.
PRISMATIC_WEIGHT = 10.0


class IkResult:

    def __init__(self, q=None, reason=None, held=(), position_error=None, aim_error=None):
        self.q = q                              # joint positions, or None if unreachable
        self.reason = reason                    # why it is unreachable, or None
        self.held = list(held)                  # joints held at their previous value, and why
        self.position_error = position_error    # meters, from the self-check
        self.aim_error = aim_error              # radians, from the self-check

    @property
    def reached(self):
        return self.q is not None


class _Failure:
    """A candidate that failed. Later stages mean it got further through the steps."""

    def __init__(self, stage, reason):
        self.stage = stage
        self.reason = reason


def _signed_angle(a, b, up):
    """Angle that turns horizontal vector a onto horizontal vector b, about up."""
    return math.atan2(up @ np.cross(a, b), a @ b)


def _wrap_into_limits(angle, near, lower, upper):
    """angle + 2*pi*n inside [lower, upper] and closest to near, or None."""
    best = None
    for n in range(-3, 4):
        value = angle + 2 * math.pi * n
        if lower - LIMIT_SLACK <= value <= upper + LIMIT_SLACK:
            if best is None or abs(value - near) < abs(best - near):
                best = value
    return None if best is None else min(max(best, lower), upper)


def _continue_from(angle, near):
    """angle + 2*pi*n closest to near: the value that keeps the joint moving smoothly."""
    return angle + 2 * math.pi * round((near - angle) / (2 * math.pi))


def _in_limits(value, lower, upper):
    return lower - LIMIT_SLACK <= value <= upper + LIMIT_SLACK


class IkSolver:

    def __init__(self, model):
        self.model = model
        m = model
        self.up = m.up
        # Yaw axis is parallel to the Theta axis, possibly flipped: yaw_axis = s * up.
        self.yaw_sign = 1.0 if m.yaw_axis @ m.up > 0 else -1.0
        # Wrist (a point on the Yaw axis) relative to the Theta axis at the reference pose.
        self.wrist_ref = m.yaw_point - m.base_point
        self.g = horizontal(self.wrist_ref, m.up)
        self.foot = -(self.g @ m.r_dir)     # R change that brings the wrist closest to the axis
        self.side_offset = m.side_offset
        self.reach = m.wrist_radius_range()

    # ----- public ------------------------------------------------------------

    def solve(self, camera, focus, previous=None):
        """Joint positions that put the camera on `camera`, looking at `focus`.

        previous: joint positions of the previous path point. Used to pick the
        closest of several solutions and to hold joints that stop mattering.
        With a previous point, Theta and Yaw must also continue smoothly from it:
        if that would take one past its limit the point is unreachable, rather
        than jumping a full turn back to the other end of its range.
        """
        m = self.model
        camera = np.asarray(camera, dtype=float)
        focus = np.asarray(focus, dtype=float)
        continuous = previous is not None
        previous = m.q_ref if previous is None else np.asarray(previous, dtype=float)
        prev_d = previous - m.q_ref
        prev_heading = prev_d[THETA] + self.yaw_sign * prev_d[YAW]

        held = []
        look = focus - camera
        distance = np.linalg.norm(look)
        if distance < SINGULAR:
            # Camera on its focus point: no aim to match, keep the old one.
            held.append('Pitch and Yaw (camera is on its focus point)')
            pitch_options = [prev_d[PITCH]]
            aim = None
        else:
            aim = look / distance
            pitch_options = self._pitch_options(aim)
            if isinstance(pitch_options, _Failure):
                return IkResult(reason=pitch_options.reason)

        candidates, failures = [], []
        for d_pitch in pitch_options:
            for item in self._solve_given_pitch(camera, aim, d_pitch, prev_d, prev_heading,
                                                continuous):
                (failures if isinstance(item, _Failure) else candidates).append(item)

        if not candidates:
            best = max(failures, key=lambda f: f.stage)
            return IkResult(reason=best.reason)

        q, cand_held = min(candidates, key=lambda c: self._distance(c[0], previous))
        return self._checked(q, camera, aim, held + cand_held)

    def solve_path(self, cameras, focuses, previous=None):
        """Solve every point in order, each starting from the previous answer.

        Stops at the first unreachable point; that failed result is the last
        item in the returned list.
        """
        results = []
        for camera, focus in zip(cameras, focuses):
            result = self.solve(camera, focus, previous)
            results.append(result)
            if not result.reached:
                break
            previous = result.q
        return results

    # ----- steps -------------------------------------------------------------

    def _pitch_options(self, aim):
        """Step 1: pitch changes that make the camera aim rise as much as `aim` does."""
        m = self.model
        u, k, look0 = m.pitch_axis, self.up, m.camera_look
        along = look0 @ u
        perp = look0 - along * u
        # k . Rot(u, a) look0 = along*(k.u) + A cos a + B sin a
        a_coef = k @ perp
        b_coef = k @ np.cross(u, perp)
        target = k @ aim - along * (k @ u)
        rho = math.hypot(a_coef, b_coef)
        # A little slack: CAD rounding (1.5708 for pi/2) makes rho just under 1,
        # which would otherwise rule out looking exactly straight up or down.
        if abs(target) > rho + 1e-6:
            return _Failure(1, 'the camera mount cannot tilt that far up or down')
        beta = math.atan2(b_coef, a_coef)
        spread = math.acos(max(-1.0, min(1.0, target / rho)))

        lo, hi = m.lower[PITCH] - m.q_ref[PITCH], m.upper[PITCH] - m.q_ref[PITCH]
        options, outside = [], []
        for d_pitch in {beta + spread, beta - spread}:
            wrapped = _wrap_into_limits(d_pitch, 0.0, lo, hi)
            if wrapped is None:
                outside.append(math.atan2(math.sin(d_pitch), math.cos(d_pitch)))
            else:
                options.append(wrapped)
        if not options:
            need = min(outside, key=abs) + m.q_ref[PITCH]
            return _Failure(1, f'aim needs Pitch {math.degrees(need):.2f} deg, limits are '
                               f'[{math.degrees(m.lower[PITCH]):.2f}, '
                               f'{math.degrees(m.upper[PITCH]):.2f}] deg')
        return options

    def _solve_given_pitch(self, camera, aim, d_pitch, prev_d, prev_heading, continuous):
        """Steps 2-7 for one pitch choice. Yields (q, held) or _Failure."""
        m = self.model
        k = self.up
        rot_pitch = axis_angle_matrix(m.pitch_axis, d_pitch)
        held = []

        # Step 2: heading = total turn about the vertical (Theta + Yaw).
        look = rot_pitch @ m.camera_look
        h_look = horizontal(look, k)
        h_aim = None if aim is None else horizontal(aim, k)
        if h_aim is None or np.linalg.norm(h_aim) < SINGULAR or np.linalg.norm(h_look) < SINGULAR:
            heading = prev_heading
            if aim is not None:
                held.append('Yaw (camera looks straight up or down)')
        else:
            heading = _signed_angle(h_look, h_aim, k)

        # Step 3: where the wrist must be, by subtracting the turned head offset.
        head = (m.pitch_point - m.yaw_point) + rot_pitch @ (m.camera_point - m.pitch_point)
        wrist = camera - m.base_point - axis_angle_matrix(k, heading) @ head

        # Height and reach are checked together so the reason lists every problem.
        problems = []

        # Step 4: Z from the wrist height.
        d_z = (k @ wrist - k @ self.wrist_ref) / (k @ m.z_dir)
        z = m.q_ref[Z] + d_z
        if not _in_limits(z, m.lower[Z], m.upper[Z]):
            word = 'high' if (d_z * (k @ m.z_dir) > 0) else 'low'
            problems.append(f'camera too {word} (Z would be {z:.4f} m, limits are '
                            f'[{m.lower[Z]:g}, {m.upper[Z]:g}])')

        # Step 5: R from the wrist's distance to the Theta axis.
        h_wrist = horizontal(wrist, k)
        radius = float(np.linalg.norm(h_wrist))
        disc = radius ** 2 - self.side_offset ** 2
        r_options = []
        if disc < -1e-12:
            problems.append(f'camera too close to the Theta axis (wrist would be '
                            f'{radius:.4f} m from it, the R slide never gets closer '
                            f'than {self.side_offset:.4f} m)')
        else:
            root = math.sqrt(max(disc, 0.0))
            r_options = [d_r for d_r in {self.foot + root, self.foot - root}
                         if _in_limits(m.q_ref[R] + d_r, m.lower[R], m.upper[R])]
            if not r_options:
                problems.append(f'camera out of reach (wrist would be {radius:.4f} m '
                                f'from the Theta axis, reach is {self.reach[0]:.4f} to '
                                f'{self.reach[1]:.4f} m)')
        if problems:
            yield _Failure(3, '; '.join(problems))
            return

        for d_r in r_options:
            r = m.q_ref[R] + d_r

            # Step 6: Theta turns the wrist onto its target direction.
            h_wrist_ref = self.g + d_r * m.r_dir
            cand_held = list(held)
            if radius < SINGULAR or np.linalg.norm(h_wrist_ref) < SINGULAR:
                d_theta = prev_d[THETA]
                cand_held.append('Theta (wrist is on the Theta axis)')
            else:
                d_theta = _signed_angle(h_wrist_ref, h_wrist, k)

            # Step 7: Yaw makes up the rest of the heading.
            d_yaw = self.yaw_sign * (heading - d_theta)

            # Step 8: place Theta and Yaw (each only known up to a full turn) in their limits.
            theta, problem = self._place_angle(THETA, m.q_ref[THETA] + d_theta,
                                               m.q_ref[THETA] + prev_d[THETA], continuous)
            if problem is None:
                yaw, problem = self._place_angle(YAW, m.q_ref[YAW] + d_yaw,
                                                 m.q_ref[YAW] + prev_d[YAW], continuous)
            if problem is not None:
                yield _Failure(5, problem)
                continue

            q = m.q_ref.copy()
            q[THETA], q[Z], q[R], q[YAW], q[PITCH] = (
                theta, z, min(max(r, m.lower[R]), m.upper[R]), yaw, m.q_ref[PITCH] + d_pitch)
            q[Z] = min(max(q[Z], m.lower[Z]), m.upper[Z])
            yield q, cand_held

    def _place_angle(self, joint, angle, near, continuous):
        """Step 8 for one angle. Returns (value, None), or (None, reason)."""
        m = self.model
        name = 'Theta' if joint == THETA else 'Yaw'
        lower, upper = m.lower[joint], m.upper[joint]
        limits = f'[{math.degrees(lower):.1f}, {math.degrees(upper):.1f}] deg'
        if continuous:
            value = _continue_from(angle, near)
            if _in_limits(value, lower, upper):
                return min(max(value, lower), upper), None
            return None, (f'{name} would pass its limit: moving on smoothly from the previous '
                          f'point needs {math.degrees(value):.1f} deg, limits are {limits} '
                          '(the only other way is a full turn back)')
        value = _wrap_into_limits(angle, near, lower, upper)
        if value is None:
            return None, f'{name} would be {math.degrees(angle):.1f} deg, limits are {limits}'
        return value, None

    # ----- checks ------------------------------------------------------------

    def _distance(self, q, previous):
        diff = q - previous
        diff[Z] *= PRISMATIC_WEIGHT
        diff[R] *= PRISMATIC_WEIGHT
        return float(diff @ diff)

    def _checked(self, q, camera, aim, held):
        position, look = self.model.camera_pose(q)
        position_error = float(np.linalg.norm(position - camera))
        aim_error = 0.0 if aim is None else math.acos(max(-1.0, min(1.0, look @ aim)))
        if position_error > POSITION_TOLERANCE or aim_error > AIM_TOLERANCE:
            return IkResult(reason=f'self-check failed (position off by {position_error:.2e} m, '
                                   f'aim off by {math.degrees(aim_error):.3f} deg); the arm '
                                   'may not match the expected layout closely enough',
                            position_error=position_error, aim_error=aim_error)
        return IkResult(q, held=held, position_error=position_error, aim_error=aim_error)
