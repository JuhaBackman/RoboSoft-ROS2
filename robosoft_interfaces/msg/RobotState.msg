# Current robot operational state
uint8 STATE_INIT=0
uint8 STATE_MAIN=1
uint8 STATE_EXIT=2

uint8 SUBSTATE_INIT_WAIT_NODES=0
uint8 SUBSTATE_INIT_WAIT_DATA=1
uint8 SUBSTATE_WAIT=8
uint8 SUBSTATE_MANUAL=9
uint8 SUBSTATE_MANUAL_RECORD=10
uint8 SUBSTATE_AUTO=11
uint8 SUBSTATE_STOP=12

uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1

std_msgs/Header header    # State time; frame is supplied by nav_msgs/Odometry
string robot_name         # Application-defined robot identifier
uint8 state_machine       # Main state: 0=INIT, 1=MAIN, 2=EXIT
uint8 substate            # 0=INIT_WAIT_NODES, 1=INIT_WAIT_DATA, 8=WAIT, 9=MANUAL, 10=MANUAL_RECORD, 11=AUTO, 12=STOP
uint8 mode                # 0=manual, 1=auto
bool is_connected         # Connected to control system
bool emergency_stop       # Emergency stop button pressed
int32 error_code          # Latest error code (0=no error)
string error_message      # Error description
