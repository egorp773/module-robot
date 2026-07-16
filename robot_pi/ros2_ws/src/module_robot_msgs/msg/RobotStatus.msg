std_msgs/Header header

uint8 STATE_BOOT=0
uint8 STATE_DISCONNECTED=1
uint8 STATE_DISARMED=2
uint8 STATE_ARMED=3
uint8 STATE_FAULT=4
uint8 STATE_ESTOP=5

uint8 state
bool connected
bool armed
bool estop
uint16 fault_code
string fault_reason
uint32 last_cmd_vel_age_ms
uint32 last_heartbeat_age_ms
uint32 last_gnss_age_ms
uint32 last_imu_age_ms
uint32 last_motor_feedback_age_ms
int16 applied_left_command
int16 applied_right_command
int16 uart_speed
int16 uart_steer
uint32 watchdog_trips
uint32 boot_counter
uint32 reset_reason
