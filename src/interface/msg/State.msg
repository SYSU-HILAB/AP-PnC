uint64 timestamp 
# state is for visualize or ROS-type understanding odometry 
float64[13] state


# mpc_state is used for mpc opitmization
# p,v,r,q
float64[13] mpc_state

float64 cur_cz

