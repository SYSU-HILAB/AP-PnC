uint64 timestamp  # time since system start [microseconds]
# Quadrotor convention: world ENU, body FLU, collective thrust along +Z body.
# p_W [m], v_W [m/s], omega_B [rad/s], q_WB [w,x,y,z].
# Hamilton unit quaternion q_WB rotates body vectors into world.
uint8 POS_START_INDEX=0
uint8 VEL_START_INDEX=3
uint8 RATE_START_INDEX=6
uint8 QUAT_START_INDEX=9
float64[13] state
