# AKI UVC/LED controller feedback decoded from proprietary PGN 0xFF31.
uint8 STATE_OFF=0
uint8 STATE_ON=1
uint8 STATE_SAFE=3

uint8 state                 # 0=OFF, 1=ON, 3=SAFE/unavailable
bool connected              # Feedback received within watchdog timeout
bool camera_light_on
bool camera_trigger_on
bool uvc_light_on
uint16 uvc_distance_front_mm
uint16 uvc_distance_back_mm
uint16 time_on_ds           # Active time in 0.1-second units
int64 timestamp             # ROS timestamp [nanoseconds]
