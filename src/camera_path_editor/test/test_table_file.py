import json

import pytest

from camera_path_editor.sampling import DEFAULT_SPEED
from camera_path_editor.table_file import (
    dumps_table, load_table, parse_table, save_table, TableFileError,
)

TEST_A_ROWS = [
    [0.6, -0.4, 0.3, 0.0, 0.0, 0.2],
    [0.6, 0.0, 0.5, 0.0, 0.0, 0.2],
    [0.6, 0.4, 0.3, 0.0, 0.0, 0.2],
]
TEST_A_SPEEDS = [0.0, 0.07, 0.0]


def test_round_trip(tmp_path):
    path = tmp_path / 'table.json'
    save_table(path, TEST_A_ROWS, TEST_A_SPEEDS, ['linear'] * 2, ['linear'] * 2)
    rows, speeds, camera_segs, focus_segs, s_curve, notes = load_table(path)
    assert rows == TEST_A_ROWS
    assert speeds == TEST_A_SPEEDS
    assert camera_segs == focus_segs == ['linear', 'linear']
    assert notes == []


def test_file_is_valid_json_with_one_row_per_line():
    text = dumps_table(TEST_A_ROWS, TEST_A_SPEEDS, ['linear'] * 2, ['linear'] * 2)
    assert json.loads(text)['version'] == 2
    assert json.loads(text)['rows'][1] == {
        'camera': [0.6, 0.0, 0.5], 'focus': [0.0, 0.0, 0.2], 'speed': 0.07}
    assert sum('"camera"' in line for line in text.splitlines()) == 3


def test_empty_table_round_trips(tmp_path):
    path = tmp_path / 'empty.json'
    save_table(path, [], [], [], [])
    assert load_table(path) == ([], [], [], [], False, [])


def test_integers_load_as_floats():
    rows, *_ = parse_table({'rows': [{'camera': [1, 2, 3], 'focus': [0, 0, 0]}]})
    assert rows == [[1.0, 2.0, 3.0, 0.0, 0.0, 0.0]]
    assert all(isinstance(v, float) for v in rows[0])


def test_missing_or_unknown_segment_types_become_linear():
    data = {
        'rows': [{'camera': [0, 0, 0], 'focus': [1, 1, 1]}] * 3,
        'camera_segment_types': ['linear', 'spline'],
    }
    _, _, camera_segs, focus_segs, _, notes = parse_table(data)
    assert camera_segs == focus_segs == ['linear', 'linear']
    assert len(notes) == 3    # two segment notes, one for the missing speeds


def test_version_1_files_get_default_speeds():
    data = {'version': 1, 'rows': [{'camera': [0, 0, i], 'focus': [1, 1, 1]} for i in range(4)]}
    _, speeds, _, _, _, notes = parse_table(data)
    assert speeds == [0.0, DEFAULT_SPEED, DEFAULT_SPEED, 0.0]
    assert any('speed' in note for note in notes)


def test_end_speeds_are_forced_to_zero_on_save_and_load():
    text = dumps_table(TEST_A_ROWS, [0.2, 0.07, 0.3], ['linear'] * 2, ['linear'] * 2)
    assert [r['speed'] for r in json.loads(text)['rows']] == TEST_A_SPEEDS
    data = json.loads(text)
    data['rows'][0]['speed'] = 0.5
    assert parse_table(data)[1] == TEST_A_SPEEDS


@pytest.mark.parametrize('data', [
    [],
    {'rows': 'nope'},
    {'rows': [{'camera': [0, 0], 'focus': [0, 0, 0]}]},
    {'rows': [{'camera': [0, 0, 'x'], 'focus': [0, 0, 0]}]},
    {'rows': [{'camera': [0, 0, True], 'focus': [0, 0, 0]}]},
    {'rows': [{'camera': [0, 0, 0]}]},
    {'rows': [[0, 0, 0, 0, 0, 0]]},
    {'rows': [{'camera': [0, 0, 0], 'focus': [0, 0, 0], 'speed': -0.1}]},
    {'rows': [{'camera': [0, 0, 0], 'focus': [0, 0, 0], 'speed': 'fast'}]},
])
def test_bad_rows_are_rejected(data):
    with pytest.raises(TableFileError):
        parse_table(data)


def test_unreadable_files_raise_table_file_error(tmp_path):
    with pytest.raises(TableFileError):
        load_table(tmp_path / 'missing.json')
    bad = tmp_path / 'bad.json'
    bad.write_text('{not json')
    with pytest.raises(TableFileError):
        load_table(bad)


def test_s_curve_round_trips_and_defaults_to_off(tmp_path):
    path = tmp_path / 's.json'
    save_table(path, TEST_A_ROWS, TEST_A_SPEEDS, ['linear'] * 2, ['linear'] * 2, s_curve=True)
    assert load_table(path)[4] is True
    data = json.loads(dumps_table(TEST_A_ROWS, TEST_A_SPEEDS, ['linear'] * 2, ['linear'] * 2))
    assert data['s_curve'] is False
    del data['s_curve']
    assert parse_table(data)[4] is False
    data['s_curve'] = 'yes'
    with pytest.raises(TableFileError):
        parse_table(data)
