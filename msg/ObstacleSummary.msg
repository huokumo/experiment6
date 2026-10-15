# All distances: horizontal distance from base_link origin, metres.
# Sector order: LEFT, FRONT, RIGHT. A clear sector means clear only in the
# observed field of view and configured obstacle height/range, not free space
# outside the camera's coverage. UNKNOWN distances are NaN.
uint8 UNKNOWN=0
uint8 CLEAR=1
uint8 BLOCKED=2
uint8 LEFT=0
uint8 FRONT=1
uint8 RIGHT=2
std_msgs/Header header
bool data_valid
string status
float64[3] distance_m
float32[3] valid_ratio
uint8[3] state
uint32 obstacle_points
