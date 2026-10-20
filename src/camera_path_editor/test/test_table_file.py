import json

import pytest

from camera_path_editor.table_file import (
    dumps_table, load_table, parse_table, save_table, TableFileError,
)

TEST_A_ROWS = [
    [0.6, -0.4, 0.3, 0.0, 0.0, 0.2],
    [0.6, 0.0, 0.5, 0.0, 0.0, 0.2],
    [0.6, 0.4, 0.3, 0.0, 0.0, 0.2],
]


def test_round_trip(tmp_path):
    path = tmp_path / 'shot.json'
    save_table(path, TEST_A_ROWS, ['linear'] * 2, ['linear'] * 2)
    rows, camera_segs, focus_segs, notes = load_table(path)
    assert rows == TEST_A_ROWS
    assert camera_segs == focus_segs == ['linear', 'linear']
    assert notes == []


def test_file_is_valid_json_with_one_row_per_line():
    text = dumps_table(TEST_A_ROWS, ['linear'] * 2, ['linear'] * 2)
    assert json.loads(text)['rows'][1] == {'camera': [0.6, 0.0, 0.5], 'focus': [0.0, 0.0, 0.2]}
    assert sum('"camera"' in line for line in text.splitlines()) == 3


def test_empty_table_round_trips(tmp_path):
    path = tmp_path / 'empty.json'
    save_table(path, [], [], [])
    assert load_table(path) == ([], [], [], [])


def test_integers_load_as_floats():
    rows, *_ = parse_table({'rows': [{'camera': [1, 2, 3], 'focus': [0, 0, 0]}]})
    assert rows == [[1.0, 2.0, 3.0, 0.0, 0.0, 0.0]]
    assert all(isinstance(v, float) for v in rows[0])


def test_missing_or_unknown_segment_types_become_linear():
    data = {
        'rows': [{'camera': [0, 0, 0], 'focus': [1, 1, 1]}] * 3,
        'camera_segment_types': ['linear', 'spline'],
    }
    _, camera_segs, focus_segs, notes = parse_table(data)
    assert camera_segs == focus_segs == ['linear', 'linear']
    assert len(notes) == 2


@pytest.mark.parametrize('data', [
    [],
    {'rows': 'nope'},
    {'rows': [{'camera': [0, 0], 'focus': [0, 0, 0]}]},
    {'rows': [{'camera': [0, 0, 'x'], 'focus': [0, 0, 0]}]},
    {'rows': [{'camera': [0, 0, True], 'focus': [0, 0, 0]}]},
    {'rows': [{'camera': [0, 0, 0]}]},
    {'rows': [[0, 0, 0, 0, 0, 0]]},
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
