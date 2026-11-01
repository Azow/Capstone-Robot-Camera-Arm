# A full camera path as typed into the editor panel.
PathRow[] rows
string[] camera_segment_types   # length = len(rows) - 1, or 0 if fewer than 2 rows
string[] focus_segment_types    # same length as camera_segment_types
# Smooth the speed changes (S-curve) so acceleration doesn't jump at rows.
# Same durations either way. Saved with the path.
bool s_curve
