# Command for robot control
uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1
uint8 MODE_STOP=2
uint8 MODE_EMERGENCY_STOP=3

uint8 ACTION_NONE=0
uint8 ACTION_START=1
uint8 ACTION_PAUSE=2
uint8 ACTION_RESUME=3
uint8 ACTION_RECORD_ON=4
uint8 ACTION_RECORD_OFF=5

uint8 IMPLEMENT_TRANSPORT=0
uint8 IMPLEMENT_HEADLAND=1
uint8 IMPLEMENT_WORKING=2

uint8 mode                # 0=manual, 1=auto, 2=stop, 3=emergency_stop
uint8 action              # 0=none, 1=start, 2=pause, 3=resume, 4=record_on, 5=record_off
float32 speed             # Desired speed [m/s] (for manual mode)
float32 steering_angle    # Desired steering angle [radians] (for manual mode)
uint8 implement_mode      # Implement control (0=transport, 1=headland, 2=working)
int32 task_id             # Task to execute (for auto mode)
int64 timestamp           # Command timestamp [unix nanoseconds]
