std_msgs/Header header

uint8 STATE_DISCONNECTED=0
uint8 STATE_DISARMED=1
uint8 STATE_MANUAL=2
uint8 STATE_AUTO=3
uint8 STATE_FAULT=4
uint8 STATE_ESTOP=5

uint8 state
string current_candidate_status
bool latched_fault
uint16 latched_fault_code
string latched_fault_reason
bool operator_armed
bool auto_armed
bool heading_initialized
bool localization_valid
bool route_valid
bool nav2_active
