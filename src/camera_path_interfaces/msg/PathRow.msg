# One row of a camera path: where the camera is, and the point it looks at.
# Coordinates are in meters in the robot base frame.
geometry_msgs/Point camera
geometry_msgs/Point focus
# Camera speed at this row in m/s (>= 0). Within a segment the speed changes at a
# constant rate from one row's speed to the next. The first and last rows are
# always treated as 0: the arm starts and ends at rest.
float64 speed
