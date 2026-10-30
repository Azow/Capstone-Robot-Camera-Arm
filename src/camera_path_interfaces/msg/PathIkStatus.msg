# IK result for the last PathTable, published by camera_path_ik for the editor panel.
# Solving stops at the first unreachable point, so at most one problem is reported.

uint32 points_total      # dense points in the path
uint32 points_reached    # reachable points from the start (= points_total if none failed)

# Where the first unreachable point is. Both are -1 when every point is reachable.
int32 row                # 0-based row, when the point is exactly on a row
int32 segment            # 0-based segment (row `segment` -> row `segment + 1`), when between rows

string reason            # why it is unreachable; empty when every point is reachable
