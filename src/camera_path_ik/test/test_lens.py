import math
from pathlib import Path

import pytest

from camera_path_ik.lens import LensConfigError, LensModel

CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'lens.yaml'


def test_shipped_config_loads():
    lens = LensModel.load(CONFIG)
    assert lens.max_angle == pytest.approx(math.radians(300))
    assert lens.min_focus == pytest.approx(0.70)


def test_ideal_is_proportional_to_inverse_distance():
    lens = LensModel.ideal('35mm', 300, 0.7)
    assert math.degrees(lens.angle(0.7)) == pytest.approx(300)
    assert math.degrees(lens.angle(1.4)) == pytest.approx(150)
    assert math.degrees(lens.angle(7.0)) == pytest.approx(30)
    assert lens.angle(math.inf) == 0.0


def test_too_close_is_clamped_to_the_close_stop():
    lens = LensModel.ideal('35mm', 300, 0.7)
    assert lens.too_close(0.5) and not lens.too_close(0.7)
    assert lens.angle(0.5) == lens.max_angle
    assert lens.angle(0.0) == lens.max_angle


def test_table_matches_ideal_when_given_ideal_points():
    ideal = LensModel.ideal('35mm', 300, 0.7)
    table = LensModel.table('35mm', [(math.inf, 0), (0.7, 300)])
    for d in (0.7, 0.9, 1.3, 2.5, 10.0, 100.0):
        assert table.angle(d) == pytest.approx(ideal.angle(d))


def test_table_interpolates_between_marks_in_inverse_distance():
    # Engraved-mark style table; points in any order.
    table = LensModel.table('marks', [(1.0, 200), (math.inf, 0), (0.7, 300), (2.0, 90)])
    assert table.min_focus == pytest.approx(0.7)
    assert math.degrees(table.angle(1.0)) == pytest.approx(200)
    # Halfway between 1/2 and 1/1 in inverse distance is 1/(0.75) = 1.333 m.
    assert math.degrees(table.angle(1 / 0.75)) == pytest.approx(145)
    assert math.degrees(table.angle(4.0)) == pytest.approx(45)


def test_sensor_offset_is_added_to_the_distance():
    lens = LensModel.ideal('35mm', 300, 0.7, sensor_offset=0.02)
    assert lens.focus_distance((0, 0, 0), (0, 1.0, 0)) == pytest.approx(1.02)


@pytest.mark.parametrize('data', [
    {'model': 'zoom'},
    {'model': 'ideal', 'throw_deg': 0, 'min_focus_m': 0.7},
    {'model': 'table', 'points': [[1.0, 10]]},
    {'model': 'table', 'points': [[1.0, 10], [2.0, 50]]},     # angle shrinks as it gets closer
    {'model': 'table', 'points': [[0.0, 10], [2.0, 5]]},
])
def test_bad_configs_are_refused(data):
    with pytest.raises(LensConfigError):
        LensModel.from_dict(data)
