"""URDFs shared by the tests."""

from pathlib import Path

REAL_URDF = Path(__file__).parents[2] / 'arm_description_v2' / 'urdf' / 'camera_arm_v2.urdf'

# A clean test arm with round numbers, so measured values can be checked by hand:
# Theta axis at the origin, R slide 0.02 m to the side of it, wrist 0.05 m out
# at R=0, camera 0.04 m in front of the Pitch axis looking along +x.
TEST_ARM = """
<robot name="test_arm">
  <link name="base_link"/> <link name="theta_link"/> <link name="z_link"/>
  <link name="r_link"/> <link name="yaw_link"/> <link name="pitch_link"/>
  <link name="camera_optical_frame"/>
  <joint name="theta" type="revolute">
    <parent link="base_link"/> <child link="theta_link"/>
    <origin xyz="0 0 0.05"/> <axis xyz="0 0 1"/> <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="z" type="prismatic">
    <parent link="theta_link"/> <child link="z_link"/>
    <origin xyz="0 0 0.1"/> <axis xyz="0 0 1"/> <limit lower="-0.1" upper="0.1"/>
  </joint>
  <joint name="r" type="prismatic">
    <parent link="z_link"/> <child link="r_link"/>
    <origin xyz="0 0.02 0"/> <axis xyz="1 0 0"/> <limit lower="0" upper="0.2"/>
  </joint>
  <joint name="yaw" type="revolute">
    <parent link="r_link"/> <child link="yaw_link"/>
    <origin xyz="0.05 0 0"/> <axis xyz="0 0 1"/> <limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="pitch" type="revolute">
    <parent link="yaw_link"/> <child link="pitch_link"/>
    <origin xyz="0 0 0.03"/> <axis xyz="0 1 0"/> <limit lower="-1.5" upper="1.0"/>
  </joint>
  <joint name="camera" type="fixed">
    <parent link="pitch_link"/> <child link="camera_optical_frame"/>
    <origin xyz="0.04 0 0" rpy="0 1.5707963267948966 0"/>
  </joint>
</robot>
"""

# The same arm, but the R slide passes through the Theta axis: the wrist
# starts 0.05 m on the far side and crosses the axis at R = 0.05. This is the
# case where Theta and Yaw line up, like the real arm fully retracted.
THROUGH_AXIS_ARM = (TEST_ARM
                    .replace('<origin xyz="0 0.02 0"/>', '<origin xyz="0 0 0"/>')
                    .replace('<origin xyz="0.05 0 0"/>', '<origin xyz="-0.05 0 0"/>'))
assert THROUGH_AXIS_ARM.count('xyz="-0.05 0 0"') == 1 and 'xyz="0 0.02 0"' not in THROUGH_AXIS_ARM

# The same arm, with Pitch able to point the camera straight up and down.
FULL_PITCH_ARM = TEST_ARM.replace('lower="-1.5" upper="1.0"', 'lower="-1.6" upper="1.6"')
assert FULL_PITCH_ARM != TEST_ARM


def sample_limits(lower, upper):
    """Joint limits to draw random test poses from: continuous joints
    (infinite limits) use one full turn, -pi to pi."""
    import numpy as np
    return (np.where(np.isfinite(lower), lower, -np.pi),
            np.where(np.isfinite(upper), upper, np.pi))
