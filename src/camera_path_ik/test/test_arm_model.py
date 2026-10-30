import numpy as np
import pytest

from camera_path_ik.arm_model import ArmLayoutError, ArmModel
from camera_path_ik.kinematics import Chain

from arms import REAL_URDF, TEST_ARM


def _assert_matches_fk(urdf_text):
    chain = Chain.from_urdf(urdf_text)
    model = ArmModel.measure(chain)
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.uniform(chain.lower, chain.upper)
        position, look = model.camera_pose(q)
        tip = chain.fk(q)
        np.testing.assert_allclose(position, tip[:3, 3], atol=1e-12)
        np.testing.assert_allclose(look, tip[:3, 2], atol=1e-12)


def test_measured_model_matches_fk_on_test_arm():
    _assert_matches_fk(TEST_ARM)


@pytest.mark.skipif(not REAL_URDF.exists(), reason='arm_description URDF not found')
def test_measured_model_matches_fk_on_real_urdf():
    _assert_matches_fk(REAL_URDF.read_text())


def test_measured_numbers_on_test_arm():
    model = ArmModel.from_urdf(TEST_ARM)
    np.testing.assert_allclose(model.base_point, [0, 0, 0.05], atol=1e-12)
    np.testing.assert_allclose(model.up, [0, 0, 1], atol=1e-12)
    assert model.side_offset == pytest.approx(0.02)
    near, far = model.wrist_radius_range()
    assert near == pytest.approx(np.hypot(0.05, 0.02))
    assert far == pytest.approx(np.hypot(0.25, 0.02))
    low, high, rise = model.camera_height_range()
    assert (low, high) == (pytest.approx(0.08), pytest.approx(0.28))
    assert rise == pytest.approx(1.0)
    np.testing.assert_allclose(model.camera_point, [0.09, 0.02, 0.18], atol=1e-12)
    np.testing.assert_allclose(model.camera_look, [1, 0, 0], atol=1e-12)


def test_wrist_range_includes_closest_point_of_slide():
    # Wrist starts 0.05 m on the far side of the axis, so sliding R from 0 to
    # 0.2 passes the axis (0.02 m to the side of it) on the way out.
    urdf = TEST_ARM.replace('<origin xyz="0.05 0 0"/>', '<origin xyz="-0.05 0 0"/>')
    near, far = ArmModel.from_urdf(urdf).wrist_radius_range()
    assert near == pytest.approx(0.02)
    assert far == pytest.approx(np.hypot(0.15, 0.02))


@pytest.mark.parametrize('old, new, message', [
    ('<joint name="z" type="prismatic">', '<joint name="z" type="revolute">', 'Z joint'),
    ('<origin xyz="0 0 0.1"/> <axis xyz="0 0 1"/>', '<origin xyz="0 0 0.1"/> <axis xyz="1 0 0"/>',
     'Z must slide parallel'),
    ('<axis xyz="1 0 0"/> <limit lower="0"', '<axis xyz="0 0 1"/> <limit lower="0"',
     'R must slide perpendicular'),
    ('<origin xyz="0.05 0 0"/> <axis xyz="0 0 1"/>', '<origin xyz="0.05 0 0"/> <axis xyz="1 0 0"/>',
     'Yaw axis must be parallel'),
    ('<axis xyz="0 1 0"/>', '<axis xyz="0 0 1"/>', 'Pitch axis must be perpendicular'),
])
def test_wrong_layout_is_rejected(old, new, message):
    assert old in TEST_ARM
    with pytest.raises(ArmLayoutError, match=message):
        ArmModel.from_urdf(TEST_ARM.replace(old, new))


def test_wrong_joint_count_is_rejected():
    urdf = TEST_ARM.replace('<joint name="pitch" type="revolute">',
                            '<joint name="pitch" type="fixed">')
    with pytest.raises(ArmLayoutError, match='Expected 5 moving joints'):
        ArmModel.from_urdf(urdf)


def test_describe_mentions_the_key_numbers():
    text = ArmModel.from_urdf(TEST_ARM).describe()
    assert '0.0539 to 0.2508 m' in text
    assert 'passes 0.0200 m from the Theta axis' in text
    assert 'outward' in text
