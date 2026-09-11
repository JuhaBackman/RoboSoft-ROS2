# Physical RoboSoft remote controller decoded from proprietary PGNs 0xFF00-0xFF02.
uint8 LAYOUT_AKI=0
uint8 LAYOUT_TURO=1

uint8 MODE_SAFE=0
uint8 MODE_MANUAL=1
uint8 MODE_AUTO_1=2
uint8 MODE_AUTO_2=3
uint8 MODE_AUTO_3=4
uint8 MODE_DIRECT_BODY=5
uint8 MODE_DIRECT_IMPLEMENT=6

uint8 layout              # 0=AKI, 1=TURO
uint8 mode                # Remote-protocol-specific mode value
bool connected            # Status PGN received within watchdog timeout
bool safety               # Remote safety circuit permits commands
bool start                # Start push button
bool deadman              # Deadman switch (AKI; true for TURO)
int32 rotary_select       # TURO rotary selector, otherwise 0
int64 timestamp           # ROS timestamp [nanoseconds]
