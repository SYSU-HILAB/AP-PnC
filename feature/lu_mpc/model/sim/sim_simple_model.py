from acados_template import AcadosModel
from casadi import SX, vertcat, reshape, vcat
from dataclasses import dataclass

@dataclass
class DroneModelConfig:
    """Configuration parameters for drone model"""
    # Time constants for first-order response
    tau_at: float = 0.1         # Thrust time constant
    tau_omega: float = 0.05     # Angular velocity time constant

    # Physical parameters
    mass: float = 1.0           # Drone mass (kg)
    g: float = -9.81            # Gravity (m/s^2)
    drag_coeff: float = 0.1     # Aerodynamic drag coefficient

# Default configuration instance
DEFAULT_CONFIG = DroneModelConfig()

def export_drone_model(config: DroneModelConfig = None):
    """
    Export drone model with configuration

    Args:
        config: DroneModelConfig instance with all parameters.
                If None, uses DEFAULT_CONFIG.
    """
    model_name = 'drone_model'

    # Use provided config or default
    config = config if config is not None else DEFAULT_CONFIG

    # Extract parameters from config
    tau_at = config.tau_at
    tau_omega = config.tau_omega
    mass = config.mass
    g = config.g
    drag_coeff = config.drag_coeff

    # --- 1. Define States ---
    p = SX.sym('p', 3)      # Position
    v = SX.sym('v', 3)      # Velocity
    # 使用四元数表示旋转 q = [qw, qx, qy, qz]
    q = SX.sym('q', 4)      
    a_t = SX.sym('a_t')     # Actual thrust
    omega = SX.sym('omega', 3) # Actual angular velocity

    x = vertcat(p, v, q, a_t, omega)

    # --- 2. Define Controls (Setpoints) ---
    a_t_ref = SX.sym('a_t_ref')
    omega_ref = SX.sym('omega_ref', 3)
    u = vertcat(a_t_ref, omega_ref)

    # --- 3. Parameters & Constants ---
    # Time constants for first-order response (now configurable parameters)
    e1 = vertcat(1, 0, 0) # 对应图片中的 e1 方向
    g = vertcat(0, 0, g)  # Gravity vector using configurable g parameter

    # --- 4. Helper Functions (Quaternion to Rotation Matrix) ---
    def quat_to_R(q):
        # standard Hamilton quaternion to SO(3)
        w, x, y, z = q[0], q[1], q[2], q[3]
        return vertcat(
            vertcat(1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w).T,
            vertcat(2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w).T,
            vertcat(2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2).T
        ).T

    R = quat_to_R(q)

    # --- 5. Aerodynamic Force (fa) ---
    # 简单的线性阻力模型: fa = -D * v_body
    # v_body = R^T * v
    v_body = R.T @ v
    fa = -drag_coeff * v_body

    # --- 6. System Dynamics (f_expl) ---
    # p_dot = v
    p_dot = v
    
    # v_dot = g + aT * R * e1 + (1/m) * R * fa
    v_dot = g + a_t * (R @ e1) + (1/mass) * (R @ fa)

    # q_dot (Quaternion dynamics)
    # q_dot = 1/2 * q * [0, omega]
    f_q = 0.5 * vertcat(
        -q[1]*omega[0] - q[2]*omega[1] - q[3]*omega[2],
         q[0]*omega[0] + q[2]*omega[2] - q[3]*omega[1],
         q[0]*omega[1] - q[1]*omega[2] + q[3]*omega[0],
         q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0]
    )

    # First-order response dynamics
    at_dot = (a_t_ref - a_t) / tau_at
    omega_dot = (omega_ref - omega) / tau_omega

    f_expl = vertcat(p_dot, v_dot, f_q, at_dot, omega_dot)

    # --- 7. Create acados model ---
    model = AcadosModel()
    model.f_expl_expr = f_expl
    model.x = x
    model.u = u
    model.name = model_name

    return model