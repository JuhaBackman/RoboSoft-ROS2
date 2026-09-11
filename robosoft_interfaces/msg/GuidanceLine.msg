# Guidance line for path tracking
uint8 LINE_HEADLAND=0
uint8 LINE_WORKING=1
uint8 LINE_TRANSPORT=2

uint8 MOVEMENT_DRIVE=0
uint8 MOVEMENT_ROTATE=1

int8 DIRECTION_REVERSE=-1
int8 DIRECTION_UNKNOWN=0
int8 DIRECTION_FORWARD=1

string name               # Line name (e.g. "A->B")
string designator         # ISO 11783 LineString/GuidancePattern designator
string guidance_group_id
string guidance_pattern_id
RoutePoint[] points       # Ordered list of route points
float32 target_speed      # Default target speed [m/s]
uint8 line_type           # 0=headland, 1=working, 2=transport
uint8 movement_type       # RoboSoft extension: 0=drive, 1=rotate
uint32 sequence           # RoboSoft extension: order inside the route
int8 driving_direction    # RoboSoft extension: -1=reverse, 0=unknown, 1=forward
bool is_active            # Is this line currently active
