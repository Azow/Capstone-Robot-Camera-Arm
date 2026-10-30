# Where camera_path_ik's playback is, published for the editor panel's play controls.

bool playing
uint32 index      # 0-based path point the arm is showing
uint32 count      # path points that can be played (the reachable ones from the start)
