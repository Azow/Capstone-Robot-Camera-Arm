# arm_bridge's view of the board, for the editor panel. Sent about 10 times a second.
bool connected
string state       # board state: IDLE, RUNNING, STOPPING, HOMING, FAULT; empty if not connected
bool homed
bool enabled
bool running       # a "Run on arm" is in progress
float64 progress   # seconds into the run, including the move to the path's start
float64 duration   # total seconds of the run
string message     # latest news for the user: errors, "Path finished", ...
