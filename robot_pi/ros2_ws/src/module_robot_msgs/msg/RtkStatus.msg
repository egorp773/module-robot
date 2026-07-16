std_msgs/Header header

uint8 CARRIER_NONE=0
uint8 CARRIER_FLOAT=1
uint8 CARRIER_FIXED=2

uint64 sensor_monotonic_us
uint8 fix_type
uint8 carrier_solution
uint8 satellites
float32 horizontal_accuracy_m
float32 vertical_accuracy_m
float32 ground_speed_mps
float32 motion_heading_rad
uint32 rtcm_age_ms
uint32 sensor_sequence
