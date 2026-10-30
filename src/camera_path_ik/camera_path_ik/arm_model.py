"""Measure the arm's dimensions from its URDF, for the closed-form IK.

Pure Python plus numpy, no ROS imports, so it can be unit tested on its own.

The IK assumes the joint layout, not the dimensions:

    Theta (turns about a vertical axis) -> Z (slides vertically)
    -> R (slides horizontally) -> Yaw (turns about a vertical axis)
    -> Pitch (turns about a horizontal axis) -> camera

"Vertical" here means parallel to the Theta axis. Everything else (lengths,
offsets, which way each joint moves, where zero is) is measured by asking the
forward kinematics where each joint's axis sits at one reference pose.
measure() also checks the layout and raises ArmLayoutError if the URDF does
not match it, so a different arm fails loudly instead of giving wrong answers.

With the measured axes, the arm can be moved without the URDF at all: start
from the camera at the reference pose and apply each joint's motion, from the
camera end inward, about (or along) its axis as it sits at the reference pose:

    camera = Theta( Z( R( Yaw( Pitch( camera at reference pose )))))

camera_pose() does exactly that, and the tests check it matches the URDF
forward kinematics. This is the "product of exponentials" form of FK.
"""

import sys

import numpy as np

from camera_path_ik.kinematics import axis_angle_matrix, Chain

# Joint order along the chain, and the type each one must be.
THETA, Z, R, YAW, PITCH = range(5)
LAYOUT = (
    ('Theta', ('revolute', 'continuous')),
    ('Z', ('prismatic',)),
    ('R', ('prismatic',)),
    ('Yaw', ('revolute', 'continuous')),
    ('Pitch', ('revolute', 'continuous')),
)

# How far from parallel/perpendicular an axis may be. CAD exports write pi/2
# as 1.5708 (about 4e-6 rad off), so exact checks would always fail.
ANGLE_TOLERANCE = 1e-3   # radians


class ArmLayoutError(Exception):
    """The URDF doesn't match the Theta -> Z -> R -> Yaw -> Pitch layout."""


def horizontal(v, up):
    """The part of v perpendicular to the Theta axis."""
    return v - (v @ up) * up


def _rotate_about(point, axis, angle, x):
    """Rotate point x by angle about the line through point along axis."""
    return point + axis_angle_matrix(axis, angle) @ (x - point)


class ArmModel:

    def __init__(self, chain, q_ref):
        self.names = chain.names
        # Copies, so remove_limits() doesn't change the chain's limits.
        self.lower = chain.lower.copy()
        self.upper = chain.upper.copy()
        self.q_ref = np.asarray(q_ref, dtype=float)

        axes = chain.joint_axes(self.q_ref)
        tip = chain.fk(self.q_ref)

        # Everything below is in base_link, at the reference pose.
        self.base_point, self.up = axes[THETA]     # Theta axis; "up" is its direction
        self.z_dir = axes[Z][1]                     # Z slides along this
        self.r_dir = axes[R][1]                     # R slides along this
        self.yaw_point, self.yaw_axis = axes[YAW]
        self.pitch_point, self.pitch_axis = axes[PITCH]
        self.camera_point = tip[:3, 3]
        self.camera_look = tip[:3, 2]               # optical z axis = viewing direction

    @classmethod
    def measure(cls, chain, q_ref=None):
        """Measure the arm at q_ref (default: all joints at 0, clamped to limits)."""
        _check_joint_types(chain)
        if q_ref is None:
            q_ref = np.clip(np.zeros(len(chain.names)), chain.lower, chain.upper)
        model = cls(chain, q_ref)
        model._check_axes()
        return model

    @classmethod
    def from_urdf(cls, urdf_text, base='base_link', tip='camera_optical_frame'):
        return cls.measure(Chain.from_urdf(urdf_text, base, tip))

    def _check_axes(self):
        def parallel(a, b):
            return np.linalg.norm(np.cross(a, b)) < ANGLE_TOLERANCE

        def perpendicular(a, b):
            return abs(a @ b) < ANGLE_TOLERANCE

        checks = (
            (parallel(self.z_dir, self.up), 'Z must slide parallel to the Theta axis'),
            (perpendicular(self.r_dir, self.up), 'R must slide perpendicular to the Theta axis'),
            (parallel(self.yaw_axis, self.up), 'Yaw axis must be parallel to the Theta axis'),
            (perpendicular(self.pitch_axis, self.up),
             'Pitch axis must be perpendicular to the Theta axis'),
            (not parallel(self.camera_look, self.pitch_axis),
             'Camera must not look along the Pitch axis (Pitch would not change the aim)'),
        )
        for ok, message in checks:
            if not ok:
                raise ArmLayoutError(message)

    def remove_limits(self, joints):
        """Let some joints move without limits (joints: indexes such as THETA, YAW).

        A testing aid: the real arm still has the URDF limits.
        """
        for joint in joints:
            self.lower[joint], self.upper[joint] = -np.inf, np.inf

    # ----- forward kinematics from the measured numbers ----------------------

    def camera_pose(self, q):
        """Camera (position, look direction) for joint positions q."""
        dq = np.asarray(q, dtype=float) - self.q_ref
        p, look = self.camera_point, self.camera_look

        p = _rotate_about(self.pitch_point, self.pitch_axis, dq[PITCH], p)
        look = axis_angle_matrix(self.pitch_axis, dq[PITCH]) @ look
        p = _rotate_about(self.yaw_point, self.yaw_axis, dq[YAW], p)
        look = axis_angle_matrix(self.yaw_axis, dq[YAW]) @ look
        p = p + dq[R] * self.r_dir
        p = p + dq[Z] * self.z_dir
        p = _rotate_about(self.base_point, self.up, dq[THETA], p)
        look = axis_angle_matrix(self.up, dq[THETA]) @ look
        return p, look

    # ----- derived numbers, for describe() and the IK ------------------------

    @property
    def side_offset(self):
        """How far the R slide line passes from the Theta axis (0 = through it)."""
        g = horizontal(self.yaw_point - self.base_point, self.up)
        return float(np.linalg.norm(g - (g @ self.r_dir) * self.r_dir))

    def wrist_radius(self, r):
        """Distance from the Theta axis to the Yaw axis with the R joint at r."""
        g = horizontal(self.yaw_point - self.base_point, self.up)
        return float(np.linalg.norm(g + (r - self.q_ref[R]) * self.r_dir))

    def wrist_radius_range(self):
        """(closest, farthest) the Yaw axis gets to the Theta axis within R's limits."""
        lo, hi = self.lower[R], self.upper[R]
        candidates = [lo, hi]
        # The closest point of the slide line to the axis, if R can reach it.
        g = horizontal(self.yaw_point - self.base_point, self.up)
        foot = self.q_ref[R] - g @ self.r_dir
        if lo < foot < hi:
            candidates.append(foot)
        radii = [self.wrist_radius(r) for r in candidates]
        return min(radii), max(radii)

    def camera_height_range(self):
        """(lowest, highest) camera height within Z's limits, at the reference head angle."""
        rise = self.z_dir @ self.up
        heights = [(self.camera_point + (z - self.q_ref[Z]) * self.z_dir) @ self.up
                   for z in (self.lower[Z], self.upper[Z])]
        return min(heights), max(heights), rise

    def describe(self):
        """Plain-text summary of the measured arm, to compare against the CAD."""
        r_near, r_far = self.wrist_radius_range()
        h_low, h_high, rise = self.camera_height_range()
        g = horizontal(self.yaw_point - self.base_point, self.up)
        r_outward = (g @ self.r_dir) > 0
        cam = self.camera_point - self.yaw_point
        cam_from_yaw = np.linalg.norm(horizontal(cam, self.up))
        cam_from_pitch = np.linalg.norm(np.cross(self.camera_point - self.pitch_point,
                                                 self.pitch_axis))
        lines = ['Joints (in chain order):']
        for (role, _), name, lo, hi in zip(LAYOUT, self.names, self.lower, self.upper):
            lines.append(f'  {role:<6} {name:<14} limits [{lo:g}, {hi:g}]')
        lines += [
            f'Theta axis: through ({self.base_point[0]:.4f}, {self.base_point[1]:.4f}), '
            f'direction ({self.up[0]:.3f}, {self.up[1]:.3f}, {self.up[2]:.3f})',
            f'Wrist (Yaw axis) distance from Theta axis: {r_near:.4f} to {r_far:.4f} m',
            f'  R slide passes {self.side_offset:.4f} m from the Theta axis; '
            f'increasing R moves the wrist {"outward" if r_outward else "inward"}',
            f'Camera height: {h_low:.4f} to {h_high:.4f} m; '
            f'increasing Z moves it {"up" if rise > 0 else "down"}',
            f'Camera offset from Yaw axis: {cam_from_yaw:.4f} m sideways, '
            f'from Pitch axis: {cam_from_pitch:.4f} m',
            f'Reference pose {np.round(self.q_ref, 4).tolist()}: camera at '
            f'{np.round(self.camera_point, 4).tolist()}, looking '
            f'{np.round(self.camera_look, 3).tolist()}',
        ]
        if r_near < 1e-3:
            lines.append('Note: the wrist can reach the Theta axis; Theta is undefined there '
                         '(Theta and Yaw line up).')
        return '\n'.join(lines)


def _check_joint_types(chain):
    if len(chain.movable) != len(LAYOUT):
        raise ArmLayoutError(
            f'Expected {len(LAYOUT)} moving joints (Theta, Z, R, Yaw, Pitch), '
            f'found {len(chain.movable)}: {chain.names}')
    for (role, types), joint in zip(LAYOUT, chain.movable):
        if joint.type not in types:
            raise ArmLayoutError(
                f'{role} joint "{joint.name}" is {joint.type}, expected {" or ".join(types)}')


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print('usage: describe_arm path/to/arm.urdf')
        return 1
    with open(argv[0]) as f:
        model = ArmModel.from_urdf(f.read())
    print(model.describe())
    return 0


if __name__ == '__main__':
    sys.exit(main())
