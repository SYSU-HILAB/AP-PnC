uint64 timestamp
# p,v,r,q
uint8 POS_START_INDEX=0
uint8 VEL_START_INDEX=3
uint8 RATE_START_INDEX=6
uint8 QUAT_START_INDEX=9
# state is for visualize or ROS-type understanding odometry
float64[13] state

float64 cur_cz
