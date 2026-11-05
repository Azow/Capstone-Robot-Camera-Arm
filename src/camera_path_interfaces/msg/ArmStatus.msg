# arm_bridge's view of the board, for the editor panel. Sent about 10 times a second.
bool connected
string state       # board state: IDLE, RUNNING, STOPPING, HOMING, FAULT; empty if not connected
bool homed
bool enabled
bool running       # a "Run on arm" is in progress
float64 progress   # seconds into the run, including the move to the path's start
float64 duration   # total seconds of the run
string message     # latest news for the user: errors, "Path finished", ...

# Per joint, in the board's order (Theta, Z, R, Yaw, Pitch). Empty if not connected.
string[] joint_names   # URDF names
float64[] positions    # rad, or m for Z and R
int32[] steps          # the board's raw step counts
float64[] lower        # soft limits from arm_bridge's config
float64[] upper
