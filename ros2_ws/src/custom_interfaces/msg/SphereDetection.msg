# SphereDetection.msg
# Message describing a detected sphere by color and image centroid

# Color constants (uint8):
uint8 COLOR_NONE   = 0  # No sphere detected
uint8 COLOR_GREEN  = 1  # Green sphere: drive forward
uint8 COLOR_ORANGE = 2  # Orange sphere: turn right
uint8 COLOR_BLUE   = 3  # Blue sphere: turn left
uint8 COLOR_YELLOW = 4  # Yellow sphere: pause

# Current detection data:
uint8   color        # One of the COLOR_* constants above
float32 centroid_x   # X-coordinate of the sphere's centroid (in pixels)
float32 centroid_y   # Y-coordinate of the sphere's centroid (in pixels)
