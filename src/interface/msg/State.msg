uint64 timestamp 
# state is for visualize or ROS-type understanding odometry 
float64[13] state

# mpc_state is used for mpc opitmization
float64[13] mpc_state


# R_state is used for mpc opitmization
# p,v,r,xb,yb  3*5
float64[3] p
float64[3] v
float64[3] r
float64[3] xb
float64[3] yb