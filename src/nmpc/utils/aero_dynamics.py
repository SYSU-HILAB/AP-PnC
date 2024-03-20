#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
import numpy as np
import casadi as cs
from casadi import SX, MX, vertcat, sin, cos,atan,asin
import math
from utils.quaternions import * 
import numpy as np
# kCL = cs.vertcat(-2.8465,   23.1516,   -0.7335,    0.1407,    0.0930,    3.8276,   -0.0006,    3.1951,   -0.0289,    0.6425,    0.0164,    0.0031,
#     0.6174,   -0.1286,    0.0342,   -0.5109,    0.0982,   -0.0006,    0.1985,   -0.0070,    0.0410,   0.0037,   -4.3496,   14.1041)

# kCD = cs.vertcat(-0.5908,   -1.8161,   -0.2045,   -0.0036,   -0.2861,   -0.8862,   -0.0590,   -0.9215,    0.0130,   -0.0035,    0.4958,   -0.0032,
#     0.0762,    0.1533,   -0.0009,    0.1891,    0.0159,    0.0172,    0.2510,    0.0032,   -0.1227,    0.1740,    0.7917,   -4.4927)

# kCY = cs.vertcat(-0.0678,    0.1151,   -0.1024,    0.6002,    0.5367,   -0.0922,   -0.8523,   -0.5220,   -0.1006,    0.2266,   -0.0345,   -0.0768,    0.0748)


kCL = np.array([-2.8465,   23.1516,   -0.7335,    0.1407,    0.0930,    3.8276,   -0.0006,    3.1951,   -0.0289,    0.6425,    0.0164,    0.0031,
    0.6174,   -0.1286,    0.0342,   -0.5109,    0.0982,   -0.0006,    0.1985,   -0.0070,    0.0410,   0.0037,   -4.3496,   14.1041])

kCD = np.array([-0.5908,   -1.8161,   -0.2045,   -0.0036,   -0.2861,   -0.8862,   -0.0590,   -0.9215,    0.0130,   -0.0035,    0.4958,   -0.0032,
    0.0762,    0.1533,   -0.0009,    0.1891,    0.0159,    0.0172,    0.2510,    0.0032,   -0.1227,    0.1740,    0.7917,   -4.4927])

kCY = np.array([-0.0678,    0.1151,   -0.1024,    0.6002,    0.5367,   -0.0922,   -0.8523,   -0.5220,   -0.1006,    0.2266,   -0.0345,   -0.0768,    0.0748])

def logistic_func(alpha, alpha1, k1, alpha2, k2):
    if k1 < 0 or k2 < 0 or alpha1 >= alpha2:
        print('Invalid logistic function')
        return None  # 这里添加了一个返回语句，以便在条件不满足时退出函数

    P1 = 1. / (1 + cs.exp(k1 * (alpha - alpha1)))
    P2 = 1. / (1 + cs.exp(-k2 * (alpha - alpha2)))
    yval = 1 - P1 - P2
    return yval

def deg2rad(deg):
    rad = deg * cs.pi / 180
    return rad
def reg_CL(alpha, beta):
    # a = alpha
    # alpha = beta
    # beta = a
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sa2 = sa ** 2
    ca2 = ca ** 2
    sb2 = sb ** 2
    cb2 = cb ** 2
    sa3 = sa ** 3
    ca3 = ca ** 3
    comp_B = cs.vertcat(cb2, alpha * cb2)
    comp_N = cs.vertcat(cb2 * ca3, ca * sb2, ca * cb2 * sa2,
                       sa * ca2 * cb2, sa * sb2, sa3 * cb2,
                       cs.fabs(sb) * cb * ca2, sa * ca * cb * cs.fabs(sb), cs.fabs(sb) * cb * sa2)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = cs.vertcat(cb2, (alpha + deg2rad(180)) * cb2)
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-20), 0.5)
    reg_N11 = w_N11 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func(alpha, deg2rad(18), 10, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = cs.vertcat(cb2, (alpha - deg2rad(180)) * cb2)
    reg_B3 = w_B3 * comp_B3

    reg =  cs.vertcat(reg_B1, reg_N11, reg_B2, reg_N2, reg_B3)

    return reg
def reg_CD(alpha, beta):
    # a = alpha
    # alpha = beta
    # beta = a
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sa2 = sa ** 2
    ca2 = ca ** 2
    sb2 = sb ** 2
    cb2 = cb ** 2
    sa3 = sa ** 3
    ca3 = ca ** 3
    comp_B = cs.vertcat(cb2, alpha * cb2)
    comp_N = cs.vertcat(cb2 * ca3, ca * sb2, ca * cb2 * sa2,
                       sa * ca2 * cb2, sa * sb2, sa3 * cb2,
                       cs.fabs(sb) * cb * ca2, sa * ca * cb * cs.fabs(sb), cs.fabs(sb) * cb * sa2)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 =cs.vertcat(cb2, (alpha + deg2rad(180)) * cb2)
    reg_B1 = w_B1 * comp_B1

    w_N1 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-10), 0.5)
    reg_N1 = w_N1 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func(alpha, deg2rad(18), 10, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 =cs.vertcat(cb2, (alpha - deg2rad(180)) * cb2)
    reg_B3 = w_B3 * comp_B3

    reg = cs.vertcat(reg_B1, reg_N1, reg_B2, reg_N2, reg_B3)

    return reg
def reg_CY(alpha, beta):
    # a = alpha
    # alpha = beta
    # beta = a
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sb2 = sb ** 2
    cb2 = cb ** 2
    signb =beta*cs.fabs(beta)
    comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = cs.vertcat(cb2, (alpha + deg2rad(180)) * cb2)
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-90), 1)
    reg_N11 = w_N11 * comp_N

    w_N12 = logistic_func(alpha, deg2rad(-90), 1, deg2rad(-11), 1)
    reg_N12 = w_N12 * comp_N

    w_N2 = logistic_func(alpha, deg2rad(20), 1, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = cs.vertcat(cb2, (alpha - deg2rad(180)) * cb2)
    reg_B3 = w_B3 * comp_B3

    reg = cs.vertcat(reg_B1, reg_N11, reg_N12, reg_N2, reg_B3)

    return reg

def caculate_f_in_body_DM(vB):
    print("vB  ",vB)
    vb_norm=math.sqrt(vB[2]*vB[2]+vB[1]*vB[1]+vB[0]*vB[0])
    va_body_z=vB[2]
    va_body_y=vB[1]
    va_body_x=vB[0]
    alpha=atan(va_body_z/va_body_x)
    beta=asin(va_body_y/vb_norm)
    print("alpha",alpha,"beta",beta)
    CL = cs.mtimes( cs.DM(kCL).T, reg_CL(alpha, beta))
    CD = cs.mtimes( cs.DM(kCD).T, reg_CD(alpha, beta))
    CY = cs.mtimes( cs.DM(kCY).T, reg_CY(alpha, beta))
    rho_s_v_v_half=0.592*vb_norm*vb_norm 
    print("rho_s_v_v_half  ",rho_s_v_v_half)
    # 创建一个 3x3 的零矩阵
    R_bv_ = np.zeros((3, 3))
    R_bv = cs.DM(R_bv_)
    # 设置矩阵 R_bs 的值，注意 Python 中的三角函数使用 radians 角度
    R_bv[0, 0] = cos(alpha)*cos(beta)
    R_bv[0, 1] = -cos(alpha)*cos(beta)
    R_bv[0, 2] = -sin(alpha )
    R_bv[1, 0] = sin(beta)
    R_bv[1, 1] = cos(beta)
    R_bv[1, 2] = 0
    R_bv[2, 0] = sin(alpha)*cos(beta)
    R_bv[2, 1] = -sin(alpha)*sin(beta)
    R_bv[2, 2] = cos(alpha)
    F_s = rho_s_v_v_half*cs.vertcat(-CD, CY, -CL)  # 这里假设 f 是一个大小为 3 的列向量
    print("f  ",F_s)

    F_b = cs.mtimes(R_bv, F_s) 
    print("f_in_trans  ",F_b)
    return F_b
def caculate_f_in_body_SX(vB):
    print("vB  ",vB)
    vb_norm=cs.sqrt(vB[2]*vB[2]+vB[1]*vB[1]+vB[0]*vB[0])
    va_body_z=vB[2]
    va_body_y=vB[1]
    va_body_x=vB[0]
    alpha=atan(va_body_z/va_body_x)
    beta=asin(va_body_y/vb_norm)
    print("vb_norm   ",vb_norm)
    print("alpha  ",alpha)
    print("beta   ",beta)
    print(" reg_CL(alpha, beta) ",type( reg_CL(alpha, beta)))
    CL = cs.mtimes( cs.SX(kCL).T, reg_CL(alpha, beta))
    CD = cs.mtimes( cs.SX(kCD).T, reg_CD(alpha, beta))
    CY = cs.mtimes( cs.SX(kCY).T, reg_CY(alpha, beta))
    rho_s_v_v_half=0.592*vb_norm*vb_norm 
    # print("rho_s_v_v_half  ",rho_s_v_v_half)
    # 创建一个 3x3 的零矩阵
    R_bv_ = np.zeros((3, 3))
    R_bv = cs.SX(R_bv_)
    # 设置矩阵 R_bs 的值，注意 Python 中的三角函数使用 radians 角度
    R_bv[0, 0] = cos(alpha)*cos(beta)
    R_bv[0, 1] = -cos(alpha)*cos(beta)
    R_bv[0, 2] = -sin(alpha )
    R_bv[1, 0] = sin(beta)
    R_bv[1, 1] = cos(beta)
    R_bv[1, 2] = 0
    R_bv[2, 0] = sin(alpha)*cos(beta)
    R_bv[2, 1] = -sin(alpha)*sin(beta)
    R_bv[2, 2] = cos(alpha)
    F_s = rho_s_v_v_half*cs.vertcat(-CD, CY, -CL)  # 这里假设 f 是一个大小为 3 的列向量
    print("R_bv type  ",type( R_bv))
    print("F_s type  ",type( F_s))
    F_b = cs.mtimes(R_bv, F_s) 
    # print("f_in_trans  ",F_b)
    return F_b
def simple_f_body_sx(vB):
    # vb_norm=cs.sqrt(vB[2]*vB[2]+vB[1]*vB[1]+vB[0]*vB[0])
    vb_norm=cs.norm_2(vB) + 1e-1
    vb_norm_pingfang=vb_norm*vb_norm
    va_body_z=vB[2]
    va_body_y=vB[1]
    va_body_x = vB[0]
    # va_body_x = 
    alpha=cs.atan2(va_body_z, va_body_x)
    beta = cs.asin(va_body_y / vb_norm)

    # print("vb_norm   ",vb_norm)
    # print("alpha  ",alpha)
    # print("beta   ",beta)
    print(" reg_CL(alpha, beta) ",type( reg_CL(alpha, beta)))
    # CL =1#+alpha*3.539
    # CD =1#+alpha*0.1910
    # CY =1
    CL = cs.mtimes( cs.SX(kCL).T, reg_CL(alpha, beta))
    CD = cs.mtimes( cs.SX(kCD).T, reg_CD(alpha, beta))
    CY = cs.mtimes( cs.SX(kCY).T, reg_CY(alpha, beta))
    rho_s_v_v_half=0.592*vb_norm_pingfang#*(vB[2]*vB[2]+vB[1]*vB[1]+vB[0]*vB[0])
    print(" rho_s_v_v_half ",type( rho_s_v_v_half))
    # print("rho_s_v_v_half  ",rho_s_v_v_half)
    # 创建一个 3x3 的零矩阵
    R_bv_ = np.zeros((3, 3))
    R_bv = cs.SX(R_bv_)

    # print("alpha",alpha)
    # print("beta",beta)
    # 设置矩阵 R_bs 的值，注意 Python 中的三角函数使用 radians 角度
    R_bv[0, 0] = cos(alpha)*cos(beta)
    R_bv[0, 1] = -cos(alpha)*cos(beta)
    R_bv[0, 2] = -sin(alpha )
    R_bv[1, 0] = sin(beta)
    R_bv[1, 1] = cos(beta)
    R_bv[1, 2] = 0
    R_bv[2, 0] = sin(alpha)*cos(beta)
    R_bv[2, 1] = -sin(alpha)*sin(beta)
    R_bv[2, 2] = cos(alpha)
    F_s = rho_s_v_v_half*cs.vertcat(-CD, CY, -CL)  # 这里假设 f 是一个大小为 3 的列向量
    # print("R_bv type  ",type( R_bv))
    # print("F_s type  ",type( F_s))
    F_b = cs.mtimes(R_bv, F_s) 
    # F_b = vB
    # print("f_in_trans  ",F_b)
    return F_b
# q = cs.SX.sym('q',4)
# # velocity [m/s]
# v = cs.SX.sym('v',3)#v_in_FLU_world
# q=[-0.707,0,0,-0.707]
# v =[0,10,0]
# linear_v_in_body = v_dot_q_inverse(v,q) 
# print(type(linear_v_in_body))
# # vB = cs.vertcat(cs.DM(50.0),cs.DM(35.0),cs.DM(10.0))
# F_b= simple_f_body_sx(linear_v_in_body)
# print(type(linear_v_in_body))
# #[w 0    x 1   y 0   z 0]  绕x转180
# q_frd2flu=np.array([0 ,1, 0, 0])
# R_frd2flu = q_to_rot_mat(q_frd2flu)
# print("R_frd2flu type  ",type( cs.SX(R_frd2flu)))
# print("R_frd2flu   ", R_frd2flu)
# F_b_flu = cs.mtimes(cs.SX(R_frd2flu), F_b)

# v_dot_q(F_b_flu/2,q)
# print("v_dot_q(F_b_flu/2,q)",v_dot_q(F_b_flu/2,q))
