from acados_template import AcadosModel
from casadi import SX, vertcat, horzcat, mtimes, inv, norm_2, cross
import numpy as np

def skew(v):
    return vertcat(
        horzcat(0, -v[2], v[1]),
        horzcat(v[2], 0, -v[0]),
        horzcat(-v[1], v[0], 0)
    )

def rodrigues_exp(v):
    """
    Axis-angle to Rotation Matrix
    """
    theta = norm_2(v) + 1e-6 # avoid singularity
    n = v / theta
    S = skew(n)
    I = SX.eye(3)
    R = I + mtimes(S, np.sin(theta)) + mtimes(mtimes(S, S), (1 - np.cos(theta)))
    return R

def inv_right_jacobian(theta_vec):
    """
    Inverse Right Jacobian approximation: I + 0.5 * skew(theta)
    """
    return SX.eye(3) + 0.5 * skew(theta_vec)

def aero_forces(v_body):
    """
    Simple linear drag model f_a = -k * v_body
    """
    k_drag = 0.1 
    return -k_drag * v_body

def export_error_state_model():
    model_name = 'enmpc_lu'

    # ---------------------------------------------
    # 1. Define Variables (Symbols)
    # ---------------------------------------------
    # Error States: delta_x
    dp = SX.sym('dp', 3)          # delta position
    dv = SX.sym('dv', 3)          # delta velocity
    dtheta = SX.sym('dtheta', 3)  # delta angle (Log mapping)
    
    x = vertcat(dp, dv, dtheta)

    # Error Controls: delta_u
    daT = SX.sym('daT', 1)        # delta thrust magnitude
    domega = SX.sym('domega', 3)  # delta angular velocity (Renamed from dw)
    
    u = vertcat(daT, domega)

    # ---------------------------------------------
    # 2. Define Parameters (Explicit Naming)
    # ---------------------------------------------
    # Trajectory References (Time-varying parameters)
    p_d = SX.sym('p_d', 3)
    v_d = SX.sym('v_d', 3)
    
    # R_d is passed as a 9x1 vector, then reshaped inside
    R_d_vec = SX.sym('R_d_vec', 9) 
    R_d = R_d_vec.reshape((3, 3))
    
    aT_d = SX.sym('aT_d', 1)
    
    # Renamed w_d -> omega_d for clarity
    omega_d = SX.sym('omega_d', 3)
    
    # Concatenate all parameters into one vector named 'params'
    # Order: [p_d(3), v_d(3), R_d(9), aT_d(1), omega_d(3)] -> Total 19 elements
    params = vertcat(p_d, v_d, R_d_vec, aT_d, omega_d)

    # Constants
    mass = 1.0  # kg
    e1 = SX([1, 0, 0]) # Thrust direction in Body Frame

    # ---------------------------------------------
    # 3. State & Input Reconstruction
    # ---------------------------------------------
    # Recover Actual State (x_act)
    # delta_v = v_d - v_act  => v_act = v_d - delta_v
    v_act = v_d - dv

    # Recover Orientation (R_act)
    # R_act = R_d * Exp(-dtheta)
    R_err_inv = rodrigues_exp(-dtheta)
    R_act = mtimes(R_d, R_err_inv)

    # Recover Inputs
    # delta_u = u_d - u_act => u_act = u_d - delta_u
    aT_act = aT_d - daT
    omega_act = omega_d - domega  # Renamed from w_act, w_d, dw

    # ---------------------------------------------
    # 4. Aerodynamics (Zero Wind)
    # ---------------------------------------------
    # Actual Aerodynamics
    v_body_act = mtimes(R_act.T, v_act) 
    f_a_act = aero_forces(v_body_act)
    
    # Reference Aerodynamics
    v_body_d = mtimes(R_d.T, v_d)
    f_a_d = aero_forces(v_body_d)

    # ---------------------------------------------
    # 5. Error Dynamics Equations
    # ---------------------------------------------
    
    # 5.1 Position Error
    dot_dp = dv
    
    # 5.2 Velocity Error
    term_d = aT_d * mtimes(R_d, e1) + (1.0/mass) * mtimes(R_d, f_a_d)
    term_act = aT_act * mtimes(R_act, e1) + (1.0/mass) * mtimes(R_act, f_a_act)
    
    dot_dv = term_d - term_act

    # 5.3 Orientation Error
    # dot_dtheta = A^T(dtheta) * (-R_d^T * R_act * omega_act + omega_d)
    
    A_T = inv_right_jacobian(dtheta)
    
    # Inner term: -Exp(-dtheta) * omega_act + omega_d
    # Note: R_d^T * R_act simplifies to R_err_inv (Exp(-dtheta))
    inner_term = -mtimes(R_err_inv, omega_act) + omega_d
    
    dot_dtheta = mtimes(A_T, inner_term)

    # ---------------------------------------------
    # 6. Finalize
    # ---------------------------------------------
    f_expl = vertcat(dot_dp, dot_dv, dot_dtheta)

    model = AcadosModel()
    model.f_expl_expr = f_expl
    model.x = x
    model.u = u
    model.p = params  # Explicitly assigned here
    model.name = model_name

    return model