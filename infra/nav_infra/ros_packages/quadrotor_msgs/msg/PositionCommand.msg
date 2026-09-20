std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
float64 yaw
float64 yaw_dot
# YAW_CONTROL_TRACK: track the commanded yaw trajectory (yaw feedforward).
# YAW_CONTROL_TARGET: command an absolute yaw target; px4ctrl drives yaw with a
# bounded shortest-path rate and the yaw reference is hidden from flatness.
uint8 YAW_CONTROL_TRACK = 0
uint8 YAW_CONTROL_TARGET = 1
uint8 yaw_control_mode
float64[3] kx
float64[3] kv 

uint32 trajectory_id

uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_READY = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 TRAJECTROY_STATUS_ABORT = 4
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7

# Its ID number will start from 1, allowing you comparing it with 0.
uint8 trajectory_flag
