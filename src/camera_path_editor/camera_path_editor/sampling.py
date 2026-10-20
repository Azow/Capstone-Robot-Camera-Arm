"""Turn a list of waypoints into a dense point list for drawing.

Pure Python, no ROS imports, so it can be unit tested on its own.

Every segment contributes exactly ``n`` points at u = 0, 1/n, ..., (n-1)/n,
and the final waypoint is appended once at the end. The camera path and the
focus path are sampled with the same ``n`` and the same u values, so point k
of one list always pairs with point k of the other.
"""

SUPPORTED_SEGMENT_TYPES = ('linear',)


def _linear(start, end, u):
    return tuple(s + u * (e - s) for s, e in zip(start, end))


def sample_path(waypoints, segment_types, n, on_unknown_type=None):
    """Return the dense list of (x, y, z) tuples for one column.

    waypoints: list of (x, y, z) tuples, one per row.
    segment_types: list of strings, one per gap (len(waypoints) - 1).
    n: points per segment, must be >= 1.
    on_unknown_type: optional callback(index, type_string) for warnings.
    """
    if len(waypoints) < 2:
        return list(waypoints)

    points = []
    for i in range(len(waypoints) - 1):
        seg_type = segment_types[i] if i < len(segment_types) else 'linear'
        if seg_type not in SUPPORTED_SEGMENT_TYPES and on_unknown_type:
            on_unknown_type(i, seg_type)
        # Only 'linear' exists in V1; unknown types fall back to it.
        start, end = waypoints[i], waypoints[i + 1]
        for k in range(n):
            points.append(_linear(start, end, k / n))
    points.append(tuple(waypoints[-1]))
    return points
