# now totally four actuators
# T, w1, w2, w3
uint64 timestamp
float64[4] motors_u
# total_thrust is always non-negative
float64 pitch_sp
# rates is in FLU
float64[3] rates_sp
float64 thrust_acc_sp
float64 sol_time

# version 1 used in px4 is total_thrust, rates_sp