# SphereDetection.msg
# Message describing a detected sphere by color and goal waypoint coordinates

# Color constants (uint8):
uint8 COLOR_NONE   = 0  # No sphere detected
uint8 COLOR_GREEN  = 1  # Green sphere: drive forward with PID controller
uint8 COLOR_ORANGE = 2  # Orange sphere: turn right with direct Twist command
uint8 COLOR_BLUE   = 3  # Blue sphere: turn left with direct Twist command
uint8 COLOR_YELLOW = 4  # Yellow sphere: pause movement temporarily

# Current detection data:
uint8   color        # One of the COLOR_* constants above
float32 x            # X-coordinate of goal waypoint for PID controller (in meters)
float32 y            # Y-coordinate of goal waypoint for PID controller (in meters)