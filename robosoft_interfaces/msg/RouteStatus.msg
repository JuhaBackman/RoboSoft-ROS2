# Generic route recording and following status.
uint8 STATE_IDLE=0
uint8 STATE_RECORDING=1
uint8 STATE_FOLLOWING=2
uint8 STATE_COMPLETED=3
uint8 STATE_ERROR=4

uint8 state
int32 guidance_index
int32 segment_index       # Active segment start; index of the previously passed route point
float32 cross_track_error # Signed distance from route [m]
float32 heading_error     # Route heading minus vehicle heading [rad]
float32 path_curvature    # Geometric segment curvature [1/km]
RoutePoint target_point     # Current projected reference on the route
RoutePoint lookahead_point  # Reference ahead by route arc distance
float32 lookahead_curvature # Curvature at lookahead point [1/km]
float32 lookahead_distance  # Arc distance from target to lookahead point [m]
string message
int64 timestamp
