from acados_template import AcadosModel
from casadi import SX, vertcat, horzcat, mtimes, norm_2
import numpy as np


def skew(v):
    return vertcat(
        horzcat(0, -v[2], v[1]), 
        horzcat(v[2], 0, -v[0]), 
        horzcat(-v[1], v[0], 0)
    )


def export_error_state_model():
    model_name = "err_state_model_lu"

    # Constants
    mass = 2.0  # kg
    e1 = SX([1, 0, 0])  # Thrust direction in Body Frame

    # ---------------------------------------------
    # 1. Define Variables (Symbols)
    # ---------------------------------------------
    # Error States: delta_x
    err_state_p = SX.sym("err_state_p", 3)  # delta position
    err_state_v = SX.sym("err_state_v", 3)  # delta velocity
    err_state_theta = SX.sym("err_state_theta", 3)  # delta angle (Log mapping)

    err_state = vertcat(err_state_p, err_state_v, err_state_theta)

    # Error Controls: delta_u
    increment_aT = SX.sym("increment_aT", 1)  # delta thrust magnitude
    increment_omega = SX.sym("increment_omega", 3)  # delta angular velocity

    u = vertcat(increment_aT, increment_omega)

    # State derivative symbols
    dot_err_state_p_sym = SX.sym("dot_err_state_p", 3)
    dot_err_state_v_sym = SX.sym("dot_err_state_v", 3)
    dot_err_state_theta_sym = SX.sym("dot_err_state_theta", 3)

    xdot = vertcat(dot_err_state_p_sym, dot_err_state_v_sym, dot_err_state_theta_sym)

    # ---------------------------------------------
    # 2. Define Parameters (Explicit Naming)
    # ---------------------------------------------
    # Trajectory References (Time-varying parameters)
    param_v_b_d = SX.sym("v_b_d", 3)

    # R_d is passed as a 9x1 vector, then reshaped inside
    param_R_d_vec = SX.sym("R_d_vec", 9)

    param_aT_d = SX.sym("aT_d", 1)

    # Renamed w_d -> omega_d for clarity
    param_omega_d = SX.sym("omega_d", 3)

    # Aerodynamic force parameters (computed externally using GD model)
    param_f_a_d = SX.sym("f_a_d", 3)    # Desired aero force (quad FLU body frame)
    param_par_f_a_par_v_b_d = SX.sym("par_f_a_par_v_b", 9)  # Partials of f_a wrt v_b (3x3 matrix flattened)

    # Concatenate all parameters into one vector named 'params'
    # Order: [v_b_d(3), R_d(9), aT_d(1), omega_d(3), f_a_d(3), par_f_a_par_v_b_d(9)]
    # Total 28 elements (was 19 before adding aero forces and Jacobian)
    params = vertcat(param_v_b_d, param_R_d_vec, param_aT_d, param_omega_d, param_f_a_d, param_par_f_a_par_v_b_d)

    # ---------------------------------------------
    # 3. Obtain Parameters Values
    # ---------------------------------------------
    val_v_b_d = param_v_b_d
    val_R_d = param_R_d_vec.reshape((3, 3))
    val_aT_d = param_aT_d
    val_omega_d = param_omega_d
    val_f_a_d = param_f_a_d
    val_par_f_a_par_v_b_d = param_par_f_a_par_v_b_d.reshape((3, 3))  # Reshape to 3x3 matrix
    # ---------------------------------------------
    # 4. Compute Jacobian Matrices
    # ---------------------------------------------
    M_v = 1/mass * val_R_d @ val_par_f_a_par_v_b_d @ val_R_d.T
    M_R = val_R_d @ (
        -val_aT_d * skew(e1) - skew(val_f_a_d / mass) + val_par_f_a_par_v_b_d @ skew(val_v_b_d/mass)
    )
    M_T = val_R_d @ e1 

    # ---------------------------------------------
    # 5. Finalize
    # ---------------------------------------------
    f_expl = vertcat(
        err_state_v,
        M_v @ err_state_v + M_R @ err_state_theta + M_T * increment_aT,
        -skew(val_omega_d) @ err_state_theta
    )
    f_impl = xdot - f_expl

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = err_state
    model.xdot = xdot
    model.u = u
    model.p = params  # Explicitly assigned here
    model.name = model_name

    return model
