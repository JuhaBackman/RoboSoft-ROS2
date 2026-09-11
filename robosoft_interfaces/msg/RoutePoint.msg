# Single point on a route/path
uint8 IMPLEMENT_TRANSPORT=0
uint8 IMPLEMENT_HEADLAND=1
uint8 IMPLEMENT_WORKING=2
uint8 IMPLEMENT_SAFE=3

float64 latitude          # WGS84 latitude [degrees]
float64 longitude         # WGS84 longitude [degrees]
float32 altitude          # Altitude [m]
float32 yaw               # Desired heading [radians]
float32 speed             # Target speed [m/s]
uint8 implement_state     # Implement control (0=transport, 1=headland, 2=working)
string designator         # ISO 11783 PointDesignator label or configured extension metadata
