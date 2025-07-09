================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: sensor_msgs/Image
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data