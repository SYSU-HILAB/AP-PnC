from acados_template import AcadosModel
from casadi import SX, vertcat, horzcat, mtimes, norm_2
import numpy as np
from ....utils.math_utils import skew_symmetric

def export_error_state_model():
    model_name = "err_state_model_lu"
    thrust_acc_ray = SX([0, 0, 1])  # Thrust direction in Body Frame
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
    # Different from IJRR 24, since it will be easier to implement.


    # R_d is passed as a 9x1 vector, then reshaped inside
    param_R_d_vec = SX.sym("R_d_vec", 9)
    # Aerodynamic force parameters (computed externally using GD model)
    # param_par_f_a_par_v_b_d = SX.sym("par_f_a_par_v_b", 9)  # Partials of f_a wrt v_b (3x3 matrix flattened)
    # REC noted that, it should use param_par_f_a_par_v_w_d instead of param_par_f_a_par_v_b
    # because this formulation will be much more easy to achieved.
    # Different from IJRR 24, since it will be easier to implement.
    param_mass_normalized_par_f_a_w_par_v_w_d = SX.sym("mass_normalized_par_f_a_w_par_v_w_d", 9)  # Partials of f_a wrt v_w (3x3 matrix flattened)
    param_aT_d = SX.sym("aT_d", 1)
    param_mass_normalized_f_a_w_d = SX.sym("mass_normalized_f_a_w_d", 3)    # Desired aero force (quad FLU body frame)
    param_v_d = SX.sym("v_d", 3)
    param_omega_d = SX.sym("omega_d", 3)



    # Concatenate all parameters into one vector named 'params'
    # Order: [R_d(9), mass_normalized_par_f_a_w_par_v_w_d(9), aT_d(1), mass_normalized_f_a_w_d(3), v_d(3), omega_d(3)]
    # Total 28 elements (was 19 before adding aero forces and Jacobian)
    params = vertcat(
        param_R_d_vec, 
        param_mass_normalized_par_f_a_w_par_v_w_d, 
        param_aT_d, 
        param_mass_normalized_f_a_w_d, 
        param_v_d, 
        param_omega_d
    )

    # ---------------------------------------------
    # 3. Obtain Parameters Values
    # ---------------------------------------------
    val_v_d = param_v_d
    val_R_d = param_R_d_vec.reshape((3, 3))
    val_aT_d = param_aT_d  # world frame  with direction.
    val_omega_d = param_omega_d
    val_mass_normalized_f_a_w_d = param_mass_normalized_f_a_w_d
    val_mass_normalized_par_f_a_w_par_v_w_d = param_mass_normalized_par_f_a_w_par_v_w_d.reshape((3, 3))  # Reshape to 3x3 matrix
    # ---------------------------------------------
    # 4. Compute Jacobian Matrices
    # ---------------------------------------------
    val_R_d_T = val_R_d.T

    M_T = val_R_d @ thrust_acc_ray 
    M_v = val_mass_normalized_par_f_a_w_par_v_w_d
    # M_R = val_R_d @ (
    #     -val_aT_d * skew_symmetric(M_T) - skew_symmetric(val_mass_normalized_f_a_w_d) + M_v @ skew_symmetric(val_v_d) 
    # )
    M_R = (
        -val_aT_d * skew_symmetric(M_T) - skew_symmetric(val_mass_normalized_f_a_w_d) + M_v @ skew_symmetric(val_v_d) 
    )@ val_R_d

    M_R = (
        val_aT_d * skew_symmetric(M_T)
    )@ val_R_d

    # ---------------------------------------------
    # 5. Finalize
    # ---------------------------------------------
    # f_expl = vertcat(
    #     err_state_v,
    #     M_v @ err_state_v + M_R @ err_state_theta,
    #     -skew_symmetric(val_omega_d) @ err_state_theta
    # ) + vertcat(
    #     SX.zeros(3, 1),
    #     M_T * increment_aT,
    #     # SX.zeros(3, 1),
    #     increment_omega,
    #     # SX.zeros(3, 1)
    # )
    f_expl = vertcat(
        err_state_v,
        M_R @ err_state_theta,
        -skew_symmetric(val_omega_d) @ err_state_theta
    ) + vertcat(
        SX.zeros(3, 1),
        M_T * increment_aT,
        # SX.zeros(3, 1),
        increment_omega,
        # SX.zeros(3, 1)
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

