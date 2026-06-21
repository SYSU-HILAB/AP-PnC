# AerodynamicData message
# Contains aerodynamic forces, moments, and state information

std_msgs/Header header

# Aerodynamic forces in body frame
geometry_msgs/Vector3 forces_body

# Aerodynamic moments in body frame
geometry_msgs/Vector3 moments_body

# Angle of attack (alpha) in radians
float64 alpha

# Sideslip angle (beta) in radians
float64 beta

# Drag coefficient state estimate
float64 cz_state

# Body frame velocity
geometry_msgs/Vector3 velocity_body
