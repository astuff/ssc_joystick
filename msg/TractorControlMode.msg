# Hexagon Tractor User Control Commands

# Joystick Levels
uint8 DEFAULT = 0
uint8 FIRST = 1
uint8 SECOND = 2
uint8 THIRD = 3
uint8 FOURTH = 4

# Hydraulic Implements
uint16 NONE = 0
uint16 YELLOW = 1
uint16 BLUE = 2
uint16 RED = 3
uint16 GREEN = 4
uint16 BROWN = 5
uint16 PINK = 6
uint16 REAR_HITCH = 7

std_msgs/Header header

uint16 joystick_mode
uint16 rpm_dial_mode
uint16 hydraulics_mode

uint8 joystick_sens
float32 rpm_dial
float32 hydraulics_in
uint16 hydraulics_implement_id
bool beacon_state_in
bool horn_state_in
