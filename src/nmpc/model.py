#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosModel
from casadi import MX, SX, vertcat, sin, cos
from utils.quaternions import *
from utils.aero_dynamics import *
import numpy as np 
import casadi as cs

# TODO MODEL PARARS TO CONFIG

def tailsitter_flu():
    model_name = "tailsitter_flu"
    # c_tau = cs.DM(0.016)
    mass = cs.DM(1.5)
    c_tau = cs.DM(0.013)
    J_data = np.array([0.0216666667, 0.0216666667, .04])
    # J_data = cs.DM([0.04000000000000001, 0.02166666666666667, 0.02166666666666667])
    J = cs.diag(J_data)
    dx = cs.DM(0.174)
    dy = cs.DM(0.174)
    # set up states & controls
    p = SX.sym('p',3)
    v = SX.sym('v',3)
    r = SX.sym('r',3)
    q = SX.sym('q',4)
    x = vertcat(p,v,r,q)
    # u is mass-normalized thrust [N/kg] >= 0, that is thrust acc
    u = SX.sym('u', 4)  
    # world frame
    g = cs.vertcat(0.0, 0.0, -9.81)
    p_dot = SX.sym("p_dot",3,1)
    v_dot = SX.sym("v_dot",3,1)
    r_dot = SX.sym("r_dot",3,1)
    q_dot = SX.sym("q_dot",4,1)
    x_dot = cs.vertcat(p_dot,v_dot,r_dot, q_dot)
    # body frame
    U = u[0] + u[1] + u[2] + u[3] 
    tb = cs.vertcat(0.0, 0.0, U)

    # aerodynamics
    vb = v_dot_q_inverse(v, q)
    
    # only aero force
    F_aero_b_frd = cs.if_else(cs.norm_2(vb) > 0.1,
        simple_f_body_sx(vb),
        cs.DM([0, 0, 0])
    )
    # F_aero_b_frd = simple_f_body_sx(vb)
    q_tf=np.array([0 ,0.707, 0, 0.707])
    R_tf= q_to_rot_mat(q_tf)
    F_aero_b_flu_quad_frame = cs.mtimes(cs.SX(R_tf), F_aero_b_frd)

    # print(F_aero_b_flu)
    # caused by tau_b 
    # data driven simulator configuration
    # 2 yb 3
    #      xb
    # 1    0
    tau_b = cs.veccat(
        dy*(-u[0]-u[1]+u[2]+u[3]),
        dx*(-u[0]+u[1]+u[2]-u[3]),
        c_tau*(-u[0]+u[1]-u[2]+u[3])
    )

    p = []

    # dynamics
    f_expl = vertcat(
        v,
        v_dot_q(tb,q)/mass + g + v_dot_q(F_aero_b_flu_quad_frame/mass,q),
        cs.mtimes(cs.inv(J), (tau_b - cs.cross(r, cs.mtimes(J, r)))),
        1 / 2 * cs.mtimes(skew_symmetric(r), q)
    )
    f_impl = x_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model


def odometry_model():
    model_name = "odometry_model"
    p = MX.sym('p',3)
    v = MX.sym('v',3)
    r = MX.sym('r',3)
    q = MX.sym('q',4)

    x = vertcat(p,q)
    
    # xdot
    p_dot = MX.sym("p_dot",3,1)
    q_dot = MX.sym("q_dot",4,1)
    x_dot = cs.vertcat(p_dot,q_dot)
    u = cs.vertcat(v, r)
    p = []
    # dynamics
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(r), q),
    )
    f_impl = x_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = p
    model.name = model_name

    return model




def tailsitter_at_w_mpc_with_no_beta() -> AcadosModel:
    model_name = "tailsitter_at_w_mpc_with_no_beta"
    mass=cs.DM(2)
    # J_data = cs.DM([0.236, 0.045,0.232])

    J_data = cs.DM([0.040,0.02166, 0.0216])
    # J_data = cs.DM([0.02166, 0.0216,0.040])
    
    J = cs.diag(J_data)


    # position [m]
    px = cs.SX.sym('px',1)
    py = cs.SX.sym('py',1)
    pz = cs.SX.sym('pz',1)
    # quaternion 
    q = cs.SX.sym('q',4)
    # velocity [m/s]
    v = cs.SX.sym('v',3)#v_in_FLU_world


    F_t = cs.SX.sym('F_t',1)
    w = cs.SX.sym('w',3)
    theta = SX.sym('theta',1)
    
    dF_t = cs.SX.sym('dF_t',1)
    dw = cs.SX.sym('dw',3)
    vtheta = SX.sym('vtheta',1)

    x = vertcat(px,py,pz,q,v,F_t,w,theta)
    u = vertcat(dF_t,dw,vtheta)

    g = cs.vertcat(0.0, 0.0, -9.81)

    # xdot
    # TODO size to 继承
    px_dot = cs.SX.sym("px_dot",1,1)
    py_dot = cs.SX.sym("py_dot",1,1)
    pz_dot = cs.SX.sym("pz_dot",1,1)
    q_dot = cs.SX.sym("q_dot",4,1)
    v_dot = cs.SX.sym("v_dot",3,1)
    F_t_dot = cs.SX.sym("F_t_dot",1,1)
    w_dot = cs.SX.sym("w_dot",3,1)
    theta_dot = cs.SX.sym("theta_dot",1,1)

    x_dot = cs.vertcat(px_dot,py_dot,pz_dot,q_dot,v_dot,F_t_dot,w_dot,theta_dot)

    
    # ft = cs.vertcat(F_t, 0.0, 0.0)
    ft = cs.vertcat( 0.0, 0.0,F_t)
    
    x_lin =  cs.SX.sym("x_lin")
    y_lin =  cs.SX.sym("y_lin")
    z_lin =  cs.SX.sym("z_lin")
    dx_by_theta_lin = cs.SX.sym("dx_by_theta_lin")
    dy_by_theta_lin = cs.SX.sym("dy_by_theta_lin")
    dz_by_theta_lin = cs.SX.sym("dz_by_theta_lin")
    theta_lin = cs.SX.sym("theta_lin")

    Qc = cs.SX.sym("Qc")
    Ql = cs.SX.sym("Ql")

    #推力角速度不要太大  
    R_aT = cs.SX.sym("R_aT")
    R_w = cs.SX.sym("R_w",3)
    J_R_w = cs.diag(R_w) 
    #d推力角速度不要太大 
    R_daT = cs.SX.sym("R_daT")
    R_dw = cs.SX.sym("R_dw",3)
    J_R_dw = cs.diag(R_dw)

    R_vtheta = cs.SX.sym("R_vtheta")
    r = cs.SX.sym("r")
    param = cs.vertcat(x_lin, y_lin, z_lin, dx_by_theta_lin, dy_by_theta_lin,dz_by_theta_lin, theta_lin, Qc, Ql, R_aT, R_w, R_daT, R_dw,R_vtheta,r)


    # dynamics  p q v
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(w), q),
        v_dot_q(ft/mass,q) + g,
        dF_t,
        dw,
        vtheta
    )
    print(f_expl)
    f_impl = x_dot - f_expl

    model = AcadosModel()


    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = param
    model.name = model_name

    x_reference = x_lin + dx_by_theta_lin * ( theta - theta_lin)
    y_reference = y_lin + dy_by_theta_lin * ( theta - theta_lin)
    z_reference = z_lin + dz_by_theta_lin * ( theta - theta_lin)
    delta_x=px-x_reference
    delta_y=py-y_reference
    delta_z=pz-z_reference
    #线性化点做切线 所以xreference的dxdydz和线性化点对齐
    dx=dx_by_theta_lin
    dy=dy_by_theta_lin
    dz=dz_by_theta_lin
    #完了 想起来这玩意没法sqrt
    e_cont_square = ((dy*delta_z-dz*delta_y)**2+(dz*delta_x-dx*delta_z)**2+(dx*delta_y-dy*delta_x)**2  )/(dx*dx+dy*dy+dz*dz)
    e_lag_square = (dx*delta_x+dy*delta_y+dz*delta_z)**2/(dx*dx+dy*dy+dz*dz)

    #无侧滑即v_in_body_y=0
    linear_v_in_body = v_dot_q_inverse(v,q)

    #linear_v_in_body[1]=0
    model.con_h_expr = vertcat(linear_v_in_body[1])

    #model.cost_expr_ext_cost = cs.mtimes(cs.mtimes(dw.T, J_R_dw),dw)+ cs.mtimes(cs.mtimes(w.T, J_R_w),w)+ e_cont_square * Qc  + e_lag_square * Ql  - R_vtheta * vtheta    #e_cont_square * Qc  + e_lag_square * Ql  - R_vtheta * vtheta + w * R_w * w + dw * R_dw * dw
    model.cost_expr_ext_cost =  e_cont_square * Qc  + e_lag_square * Ql    - R_vtheta * vtheta +cs.mtimes(cs.mtimes(dw.T, J_R_dw),dw)+ cs.mtimes(cs.mtimes(w.T, J_R_w),w)+ dF_t*R_daT*dF_t
    return model

def tailsitter_at_w_mpc_with_no_beta_with_air() -> AcadosModel:
    model_name = "tailsitter_at_w_mpc_with_no_beta_with_air"
    mass=cs.DM(2)
    # J_data = cs.DM([0.236, 0.045,0.232])

    J_data = cs.DM([0.040,0.02166, 0.0216])
    # J_data = cs.DM([0.02166, 0.0216,0.040])
    
    J = cs.diag(J_data)


    # position [m]
    px = cs.SX.sym('px',1)
    py = cs.SX.sym('py',1)
    pz = cs.SX.sym('pz',1)
    # quaternion 
    q = cs.SX.sym('q',4)
    # velocity [m/s]
    v = cs.SX.sym('v',3)#v_in_FLU_world


    F_t = cs.SX.sym('F_t',1)
    w = cs.SX.sym('w',3)
    theta = SX.sym('theta',1)
    
    dF_t = cs.SX.sym('dF_t',1)
    dw = cs.SX.sym('dw',3)
    vtheta = SX.sym('vtheta',1)

    x = vertcat(px,py,pz,q,v,F_t,w,theta)
    u = vertcat(dF_t,dw,vtheta)

    g = cs.vertcat(0.0, 0.0, -9.81)

    # xdot
    # TODO size to 继承
    px_dot = cs.SX.sym("px_dot",1,1)
    py_dot = cs.SX.sym("py_dot",1,1)
    pz_dot = cs.SX.sym("pz_dot",1,1)
    q_dot = cs.SX.sym("q_dot",4,1)
    v_dot = cs.SX.sym("v_dot",3,1)
    F_t_dot = cs.SX.sym("F_t_dot",1,1)
    w_dot = cs.SX.sym("w_dot",3,1)
    theta_dot = cs.SX.sym("theta_dot",1,1)

    x_dot = cs.vertcat(px_dot,py_dot,pz_dot,q_dot,v_dot,F_t_dot,w_dot,theta_dot)

    
    # ft = cs.vertcat(F_t, 0.0, 0.0)
    ft = cs.vertcat( 0.0, 0.0,F_t)
    
    x_lin =  cs.SX.sym("x_lin")
    y_lin =  cs.SX.sym("y_lin")
    z_lin =  cs.SX.sym("z_lin")
    dx_by_theta_lin = cs.SX.sym("dx_by_theta_lin")
    dy_by_theta_lin = cs.SX.sym("dy_by_theta_lin")
    dz_by_theta_lin = cs.SX.sym("dz_by_theta_lin")
    theta_lin = cs.SX.sym("theta_lin")

    Qc = cs.SX.sym("Qc")
    Ql = cs.SX.sym("Ql")

    #推力角速度不要太大  
    R_aT = cs.SX.sym("R_aT")
    R_w = cs.SX.sym("R_w",3)
    J_R_w = cs.diag(R_w) 
    #d推力角速度不要太大 
    R_daT = cs.SX.sym("R_daT")
    R_dw = cs.SX.sym("R_dw",3)
    J_R_dw = cs.diag(R_dw)

    R_vtheta = cs.SX.sym("R_vtheta")
    r = cs.SX.sym("r")
    param = cs.vertcat(x_lin, y_lin, z_lin, dx_by_theta_lin, dy_by_theta_lin,dz_by_theta_lin, theta_lin, Qc, Ql, R_aT, R_w, R_daT, R_dw,R_vtheta,r)

    #无侧滑即v_in_body_y=0
    linear_v_in_body = v_dot_q_inverse(v,q) #v_in_FLU_body

    print("vB  ",linear_v_in_body)
    F_b_frd = simple_f_body_sx(linear_v_in_body)

    q_frd2flu=np.array([0 ,1, 0, 0])
    R_frd2flu = q_to_rot_mat(q_frd2flu)
    F_b_flu = cs.mtimes(cs.SX(R_frd2flu), F_b)
    # dynamics  p q v #v_in_FLU_world
    f_expl = vertcat(
        v,
        1 / 2 * cs.mtimes(skew_symmetric(w), q),
        v_dot_q(ft/mass,q) + g + v_dot_q(F_b_flu/mass,q),
        dF_t,
        dw,
        vtheta
    )
    print(f_expl)
    f_impl = x_dot - f_expl

    model = AcadosModel()


    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    # model.z = z
    model.p = param
    model.name = model_name

    x_reference = x_lin + dx_by_theta_lin * ( theta - theta_lin)
    y_reference = y_lin + dy_by_theta_lin * ( theta - theta_lin)
    z_reference = z_lin + dz_by_theta_lin * ( theta - theta_lin)
    delta_x=px-x_reference
    delta_y=py-y_reference
    delta_z=pz-z_reference
    #线性化点做切线 所以xreference的dxdydz和线性化点对齐
    dx=dx_by_theta_lin
    dy=dy_by_theta_lin
    dz=dz_by_theta_lin
    #完了 想起来这玩意没法sqrt
    e_cont_square = ((dy*delta_z-dz*delta_y)**2+(dz*delta_x-dx*delta_z)**2+(dx*delta_y-dy*delta_x)**2  )/(dx*dx+dy*dy+dz*dz)
    e_lag_square = (dx*delta_x+dy*delta_y+dz*delta_z)**2/(dx*dx+dy*dy+dz*dz)



    #linear_v_in_body[1]=0
    model.con_h_expr = vertcat(linear_v_in_body[1])#e_cont_square+e_lag_square-(r)**2 #

    #model.cost_expr_ext_cost = cs.mtimes(cs.mtimes(dw.T, J_R_dw),dw)+ cs.mtimes(cs.mtimes(w.T, J_R_w),w)+ e_cont_square * Qc  + e_lag_square * Ql  - R_vtheta * vtheta    #e_cont_square * Qc  + e_lag_square * Ql  - R_vtheta * vtheta + w * R_w * w + dw * R_dw * dw
    model.cost_expr_ext_cost =  e_cont_square * Qc  + e_lag_square * Ql    - R_vtheta * vtheta +cs.mtimes(cs.mtimes(dw.T, J_R_dw),dw)+ cs.mtimes(cs.mtimes(w.T, J_R_w),w)+ dF_t*R_daT*dF_t
    return model

def huber_function(x, delta):
  """
  Defines the Huber function.

  Args:
      x: Symbolic expression representing the error.
      delta: Symbolic expression representing the delta parameter.

  Returns:
      A symbolic expression representing the Huber loss.
  """
  squared_term = x*x
  linear_term = 2*delta*cs.fabs(x) - delta**2
  return cs.if_else(cs.fabs(x) <= delta, squared_term, linear_term)

def test_huber():
    # vb
    x = cs.SX.sym('x',3,1)

# # 定义 Huber 函数的参数
#     delta = 1.0

# # 计算 Huber 函数
#     hf = huber_function(x, delta)
#     print('----------------------')
#     print(hf)
    print('----------------------')
    print(q_to_rot_mat(np.array([0,0.707,0, 0.707])))
    print('----------------------')
    ff = cs.Function('ff', [x], [simple_f_body_sx(x)])
    vb = cs.SX([10.0, 0.0, 1.0])

    
    print(caculate_f_in_body_SX(vb))

if __name__ == '__main__':
    model = test_huber()
    # print(model.)

