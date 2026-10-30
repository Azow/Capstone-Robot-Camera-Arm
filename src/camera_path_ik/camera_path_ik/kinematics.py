"""Forward kinematics for a serial chain read straight from a URDF file.

Pure Python plus numpy, no ROS imports, so it can be unit tested on its own.
Reading the joints from the URDF (instead of retyping the numbers) means the
IK always matches what robot_state_publisher draws in RViz.
"""

import xml.etree.ElementTree as ET

import numpy as np


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def rpy_matrix(roll, pitch, yaw):
    """URDF rotation: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_matrix(axis, angle):
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    t = 1.0 - c
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def transform(rotation, translation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


class Joint:

    def __init__(self, name, joint_type, origin, axis, lower, upper):
        self.name = name
        self.type = joint_type          # 'revolute', 'prismatic' or 'fixed'
        self.origin = origin            # 4x4 parent -> joint frame
        self.axis = axis                # unit vector in the joint frame
        self.lower = lower
        self.upper = upper

    @property
    def movable(self):
        return self.type in ('revolute', 'prismatic', 'continuous')

    def motion(self, q):
        """4x4 transform caused by moving this joint to position q."""
        if self.type in ('revolute', 'continuous'):
            return transform(axis_angle_matrix(self.axis, q), np.zeros(3))
        if self.type == 'prismatic':
            return transform(np.eye(3), self.axis * q)
        return np.eye(4)


class Chain:
    """The joints from base_link to the tip link, in order."""

    def __init__(self, joints):
        self.joints = joints
        self.movable = [j for j in joints if j.movable]
        self.names = [j.name for j in self.movable]
        self.lower = np.array([j.lower for j in self.movable])
        self.upper = np.array([j.upper for j in self.movable])

    @classmethod
    def from_urdf(cls, urdf_text, base='base_link', tip='camera_optical_frame'):
        root = ET.fromstring(urdf_text)
        by_child = {}
        for el in root.findall('joint'):
            origin = el.find('origin')
            xyz = _floats(origin.get('xyz') if origin is not None else None, (0, 0, 0))
            rpy = _floats(origin.get('rpy') if origin is not None else None, (0, 0, 0))
            axis_el = el.find('axis')
            axis = np.array(_floats(axis_el.get('xyz') if axis_el is not None else None,
                                    (1, 0, 0)))
            axis = axis / np.linalg.norm(axis)
            limit = el.find('limit')
            lower = float(limit.get('lower', '-inf')) if limit is not None else -np.inf
            upper = float(limit.get('upper', 'inf')) if limit is not None else np.inf
            if el.get('type') == 'continuous':
                lower, upper = -np.inf, np.inf
            joint = Joint(el.get('name'), el.get('type'),
                          transform(rpy_matrix(*rpy), xyz), axis, lower, upper)
            by_child[el.find('child').get('link')] = (el.find('parent').get('link'), joint)

        joints = []
        link = tip
        while link != base:
            if link not in by_child:
                raise ValueError(f'No joint chain from {base} to {tip} in the URDF')
            link, joint = by_child[link]
            joints.append(joint)
        joints.reverse()
        return cls(joints)

    def fk(self, q):
        """4x4 pose of the tip in the base frame for joint positions q."""
        m = np.eye(4)
        values = iter(q)
        for joint in self.joints:
            m = m @ joint.origin
            if joint.movable:
                m = m @ joint.motion(next(values))
        return m

    def joint_axes(self, q):
        """(point, direction) of each movable joint's axis in the base frame at pose q.

        A revolute joint turns about the line through point along direction.
        A prismatic joint slides along direction (point is where its frame sits).
        """
        m = np.eye(4)
        values = iter(q)
        axes = []
        for joint in self.joints:
            m = m @ joint.origin
            if joint.movable:
                axes.append((m[:3, 3].copy(), m[:3, :3] @ joint.axis))
                m = m @ joint.motion(next(values))
        return axes
