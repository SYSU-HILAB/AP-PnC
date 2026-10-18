uint64 timestamp		# time since system start (microseconds)

# Positions [m] and velocities [m/s] are world ENU.
float32[3] ref_position
float32[3] ref_velocity
float32[3] actual_position
float32[3] actual_velocity
float32 ref_vxy
float32 actual_vxy
# Body-Y unit directions expressed in world ENU.
float32[3] ref_yb
float32[3] actual_yb
# Body-FLU angles [rad]; v_B = R_WB^T v_W (controller assumes zero wind).
# alpha = atan2(v_B.x,v_B.z); beta = atan2(v_B.y,hypot(v_B.x,v_B.z)).
float32 alpha
float32 beta
# IMU-derived, mass-normalized body-X coefficient [1/m].
float32 cx
