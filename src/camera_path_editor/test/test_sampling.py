from camera_path_editor.sampling import sample_path

TEST_A_CAMERA = [(0.6, -0.4, 0.3), (0.6, 0.0, 0.5), (0.6, 0.4, 0.3)]
TEST_A_FOCUS = [(0.0, 0.0, 0.2)] * 3


def test_point_count_matches_formula():
    for n in (1, 5, 20):
        cam = sample_path(TEST_A_CAMERA, ['linear'] * 2, n)
        foc = sample_path(TEST_A_FOCUS, ['linear'] * 2, n)
        assert len(cam) == len(foc) == 2 * n + 1


def test_path_hits_every_waypoint():
    n = 20
    cam = sample_path(TEST_A_CAMERA, ['linear'] * 2, n)
    for i, wp in enumerate(TEST_A_CAMERA):
        assert cam[i * n] == wp


def test_midpoint_is_linear():
    cam = sample_path([(1.0, 0.0, 0.3), (1.0, 1.0, 0.3)], ['linear'], 20)
    assert cam[10] == (1.0, 0.5, 0.3)


def test_zero_and_one_rows():
    assert sample_path([], [], 20) == []
    assert sample_path([(1.0, 2.0, 3.0)], [], 20) == [(1.0, 2.0, 3.0)]


def test_unknown_type_falls_back_to_linear_and_warns():
    warnings = []
    pts = sample_path([(0, 0, 0), (1, 0, 0)], ['spline'], 4,
                      lambda i, t: warnings.append((i, t)))
    assert warnings == [(0, 'spline')]
    assert pts == [(0, 0, 0), (0.25, 0, 0), (0.5, 0, 0), (0.75, 0, 0), (1, 0, 0)]
