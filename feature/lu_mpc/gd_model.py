# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@outlook.com
#
import math
from venv import create

import casadi as cs
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from casadi import MX, SX, asin, atan, atan2, cos, sin, vertcat
from scipy.spatial.transform import Rotation

kCL = np.array(
    [
        -2.8465,
        23.1516,
        -0.7335,
        0.1407,
        0.0930,
        3.8276,
        -0.0006,
        3.1951,
        -0.0289,
        0.6425,
        0.0164,
        0.0031,
        0.6174,
        -0.1286,
        0.0342,
        -0.5109,
        0.0982,
        -0.0006,
        0.1985,
        -0.0070,
        0.0410,
        0.0037,
        -4.3496,
        14.1041,
    ]
)

kCD = np.array(
    [
        -0.5908,
        -1.8161,
        -0.2045,
        -0.0036,
        -0.2861,
        -0.8862,
        -0.0590,
        -0.9215,
        0.0130,
        -0.0035,
        0.4958,
        -0.0032,
        0.0762,
        0.1533,
        -0.0009,
        0.1891,
        0.0159,
        0.0172,
        0.2510,
        0.0032,
        -0.1227,
        0.1740,
        0.7917,
        -4.4927,
    ]
)

kCY = np.array(
    [
        -0.0678,
        0.1151,
        -0.1024,
        0.6002,
        0.5367,
        -0.0922,
        -0.8523,
        -0.5220,
        -0.1006,
        0.2266,
        -0.0345,
        -0.0768,
        0.0748,
    ]
)

kCLL = np.array(
    [
        -0.0240,
        0.0172,
        -0.0219,
        -0.0085,
        -0.2163,
        -0.0143,
        -0.0152,
        -0.1874,
        -0.0021,
        -0.0065,
        -0.0110,
        0.0029,
        -0.0184,
        -0.0028,
        0.0203,
        -0.0158,
        -0.0245,
        -0.0068,
    ]
)

kCm = np.array(
    [
        -1.9963,
        -2.5059,
        0.5987,
        0.0120,
        0.7977,
        0.0543,
        0.6310,
        -0.1122,
        -0.0798,
        -0.2177,
        -0.1059,
        0.0029,
        -0.1388,
        0.0132,
        -0.2605,
        0.0015,
        -0.5789,
        0.7902,
    ]
)

kCn = np.array(
    [
        0.0681,
        -0.0338,
        0.0054,
        0.0028,
        -0.0052,
        0.0034,
        -0.0048,
        0.0217,
        0.0111,
        0.0121,
        0.0685,
        0.0240,
    ]
)


def logistic_func(alpha, alpha1, k1, alpha2, k2):
    if k1 < 0 or k2 < 0 or alpha1 >= alpha2:
        print("Invalid logistic function")
        return None  # 这里添加了一个返回语句，以便在条件不满足时退出函数

    P1 = 1.0 / (1 + cs.exp(k1 * (alpha - alpha1)))
    P2 = 1.0 / (1 + cs.exp(-k2 * (alpha - alpha2)))
    yval = 1 - P1 - P2
    return yval


def deg2rad(deg):
    rad = deg * cs.pi / 180
    return rad


def reg_CL(alpha, beta):
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2
    sa3 = sa**3
    ca3 = ca**3
    comp_B = cs.vertcat(cb2, alpha * cb2)
    comp_N = cs.vertcat(
        cb2 * ca3,
        ca * sb2,
        ca * cb2 * sa2,
        sa * ca2 * cb2,
        sa * sb2,
        sa3 * cb2,
        cs.fabs(sb) * cb * ca2,
        sa * ca * cb * cs.fabs(sb),
        cs.fabs(sb) * cb * sa2,
    )

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

    reg = cs.vertcat(reg_B1, reg_N11, reg_B2, reg_N2, reg_B3)

    return reg


def reg_CD(alpha, beta):
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2
    sa3 = sa**3
    ca3 = ca**3
    comp_B = cs.vertcat(cb2, alpha * cb2)
    comp_N = cs.vertcat(
        cb2 * ca3,
        ca * sb2,
        ca * cb2 * sa2,
        sa * ca2 * cb2,
        sa * sb2,
        sa3 * cb2,
        cs.fabs(sb) * cb * ca2,
        sa * ca * cb * cs.fabs(sb),
        cs.fabs(sb) * cb * sa2,
    )

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = cs.vertcat(cb2, (alpha + deg2rad(180)) * cb2)
    reg_B1 = w_B1 * comp_B1

    w_N1 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-10), 0.5)
    reg_N1 = w_N1 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func(alpha, deg2rad(18), 10, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = cs.vertcat(cb2, (alpha - deg2rad(180)) * cb2)
    reg_B3 = w_B3 * comp_B3

    reg = cs.vertcat(reg_B1, reg_N1, reg_B2, reg_N2, reg_B3)

    return reg


def reg_CY(alpha, beta):
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    signb = beta * cs.fabs(beta)
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


def reg_Cll(alpha, beta):
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    comp_B = cs.vertcat(cb2, alpha * cb2)
    signb = beta * cs.fabs(beta)
    comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = cs.vertcat(cb2, (alpha + deg2rad(180)) * cb2)
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-90), 5)
    reg_N11 = w_N11 * comp_N

    w_N12 = logistic_func(alpha, deg2rad(-90), 5, deg2rad(-20), 0.5)
    reg_N12 = w_N12 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-11), 20, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N22 = logistic_func(alpha, deg2rad(18), 10, deg2rad(90), 5)
    reg_N22 = w_N22 * comp_N

    w_N2 = logistic_func(alpha, deg2rad(90), 5, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = cs.vertcat(cb2, (alpha - deg2rad(180)) * cb2)
    reg_B3 = w_B3 * comp_B3

    reg = cs.vertcat(reg_B1, reg_N11, reg_N12, reg_B2, reg_N2, reg_N22, reg_B3)

    return reg


def reg_Cm(alpha, beta):
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2
    comp_B = cs.vertcat(cb2, alpha * cb2)
    comp_N = cs.vertcat(cb2 * ca2, sb2, cb2 * sa2, cs.fabs(sb) * cb * ca, sa * ca * cb2, cs.fabs(sb) * cb * sa)

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

    reg = cs.vertcat(reg_B1, reg_N11, reg_B2, reg_N2, reg_B3)

    return reg


def reg_Cn(alpha, beta):
    sa = sin(alpha)
    ca = cos(alpha)
    sb = sin(beta)
    cb = cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    comp_B = cs.vertcat(cb2, alpha * cb2)
    signb = beta * cs.fabs(beta)
    comp_N = cs.vertcat(signb * sb2, sb * cb * ca, sb * cb * sa)

    w_B1 = logistic_func(alpha, deg2rad(-190), 1, deg2rad(-170), 1)
    comp_B1 = cs.vertcat(cb2, (alpha + deg2rad(180)) * cb2)
    reg_B1 = w_B1 * comp_B1

    w_N1 = logistic_func(alpha, deg2rad(-170), 1, deg2rad(-10), 10)
    reg_N1 = w_N1 * comp_N

    w_B2 = logistic_func(alpha, deg2rad(-10), 10, deg2rad(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func(alpha, deg2rad(0), 0.5, deg2rad(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func(alpha, deg2rad(170), 1, deg2rad(190), 1)
    comp_B3 = cs.vertcat(cb2, (alpha - deg2rad(180)) * cb2)
    reg_B3 = w_B3 * comp_B3

    reg = cs.vertcat(reg_B1, reg_N1, reg_B2, reg_N2, reg_B3)

    return reg


def aerodynamics_body_SX(vB):
    """
    Calculate aerodynamic forces in body frame using SX (Symbolic Expression) evaluation.
    
    Args:
        vB (cs.SX): Velocity vector in body frame.

    Returns:
        cs.SX: Aerodynamic forces in body frame.
    """
    vb_norm = cs.sqrt(vB[2] * vB[2] + vB[1] * vB[1] + vB[0] * vB[0])
    va_body_z = vB[2]
    va_body_y = vB[1]
    va_body_x = vB[0]
    alpha = atan(va_body_z / va_body_x)
    beta = asin(va_body_y / vb_norm)
    CL = cs.mtimes(cs.SX(kCL).T, reg_CL(alpha, beta))
    CD = cs.mtimes(cs.SX(kCD).T, reg_CD(alpha, beta))
    CY = cs.mtimes(cs.SX(kCY).T, reg_CY(alpha, beta))
    # 0.592 is 1/2 * rho * S
    rho_s_v_v_half = 0.592 * vb_norm * vb_norm
    # 创建一个 3x3 的零
    R_bs = cs.SX.zeros(3, 3)
    R_bs[0, 0] = cos(alpha)
    R_bs[0, 2] = -sin(alpha)
    R_bs[1, 1] = 1.0
    R_bs[2, 0] = sin(alpha)
    R_bs[2, 2] = cos(alpha)
    F_s = rho_s_v_v_half * cs.vertcat(
        -CD, CY, -CL
    )  
    F_b = cs.mtimes(R_bs, F_s)
    return F_b

def aerodynamics_body_vel_scaled_SX(vB):
    """
    Calculate aerodynamic forces in body frame using SX (Symbolic Expression) evaluation.
    
    Args:
        vB (cs.SX): Velocity vector in body frame.

    Returns:
        cs.SX: Aerodynamic forces in body frame.
    """
    vb_norm = cs.sqrt(vB[2] * vB[2] + vB[1] * vB[1] + vB[0] * vB[0])
    va_body_z = vB[2]
    va_body_y = vB[1]
    va_body_x = vB[0]
    alpha = atan(va_body_z / va_body_x)
    beta = asin(va_body_y / vb_norm)
    CL = cs.mtimes(cs.SX(kCL).T, reg_CL(alpha, beta))
    CD = cs.mtimes(cs.SX(kCD).T, reg_CD(alpha, beta))
    CY = cs.mtimes(cs.SX(kCY).T, reg_CY(alpha, beta))
    # 0.592 is 1/2 * rho
    rho_s_v_v_half = 0.592 * vb_norm * vb_norm
    # 创建一个 3x3 的零
    R_bs = cs.SX.zeros(3, 3)
    R_bs[0, 0] = cos(alpha)
    R_bs[0, 2] = -sin(alpha)
    R_bs[1, 1] = 1.0
    R_bs[2, 0] = sin(alpha)
    R_bs[2, 2] = cos(alpha)
    F_s = rho_s_v_v_half * cs.vertcat(
        -CD, CY, -CL
    )  
    F_b = cs.mtimes(R_bs, F_s)
    return F_b

def aerodynamics_forces_moments_SX(vB):
    """
    Calculate aerodynamic forces and moments using the complete DYL model.

    Args:
        vB (cs.SX): Velocity vector in body frame.

    Returns:
        tuple: (F_b, M_b) - forces and moments in body frame
    """
    eps = 1e-6
    vb_norm = cs.sqrt(vB[0]**2 + vB[1]**2 + vB[2]**2)

    # Check for zero velocity
    zero_vel = vb_norm < eps

    # Calculate angles
    vx = vB[0]
    vy = vB[1]
    vz = vB[2]

    alpha = atan2(vz, vx)
    beta = asin(vy / vb_norm)

    # Air density and dynamic pressure
    rho = 1.225
    P = 0.5 * rho * vb_norm**2

    # Force coefficients
    CL = cs.mtimes(cs.SX(kCL).T, reg_CL(alpha, beta))
    CD = cs.mtimes(cs.SX(kCD).T, reg_CD(alpha, beta))
    CY = cs.mtimes(cs.SX(kCY).T, reg_CY(alpha, beta))

    # Moment coefficients
    Cl = cs.mtimes(cs.SX(kCLL).T, reg_Cll(alpha, beta))
    Cm = cs.mtimes(cs.SX(kCm).T, reg_Cm(alpha, beta))
    Cn = cs.mtimes(cs.SX(kCn).T, reg_Cn(alpha, beta))

    # Forces in stability frame (drag, side, lift)
    F_s = P * cs.vertcat(-CD, CY, -CL)

    # Transformation matrix from stability to body frame
    R_bs = cs.SX.zeros(3, 3)
    R_bs[0, 0] = cos(alpha)
    R_bs[0, 2] = -sin(alpha)
    R_bs[1, 1] = 1.0
    R_bs[2, 0] = sin(alpha)
    R_bs[2, 2] = cos(alpha)

    # Forces in body frame
    F_b = cs.mtimes(R_bs, F_s)

    # Moments in body frame (rolling, pitching, yawing)
    M_b = P * cs.vertcat(Cl, Cm, Cn)

    # Handle zero velocity case
    F_b = cs.if_else(zero_vel, cs.SX.zeros(3), F_b)
    M_b = cs.if_else(zero_vel, cs.SX.zeros(3), M_b)

    return F_b, M_b


def create_aerodynamics_func():
    """Function to calculate aerodynamic forces using DM (Direct Method) evaluation"""
    # Create symbolic variables and function for evaluation
    vB_sym = cs.SX.sym("vB", 3)
    aero_func = cs.Function("aero_force", [vB_sym], [aerodynamics_body_SX(vB_sym)])

    # Evaluate with the given DM input
    return aero_func


def create_aerodynamics_forces_moments_func():
    """Function to calculate aerodynamic forces and moments using DM evaluation"""
    vB_sym = cs.SX.sym("vB", 3)
    F_b, M_b = aerodynamics_forces_moments_SX(vB_sym)
    aero_func = cs.Function("aero_forces_moments", [vB_sym], [F_b, M_b])

    return aero_func


def plot_cl_cd_aero():
    """
    Plot CL and CD coefficients across AoA from -180° to 180° at 12 m/s velocity norm.
    """
    # Set velocity parameters
    v_norm = 12.0  # m/s
    beta = 0.0  # sideslip angle in radians

    # Generate AoA range from -180 to 180 degrees
    aoa_deg = np.linspace(-180, 180, 361)  # 1-degree increments
    aoa_rad = np.deg2rad(aoa_deg)

    # Calculate CL and CD for each AoA
    cl_values = []
    cd_values = []

    for alpha in aoa_rad:
        # Calculate regression terms
        reg_cl = reg_CL(alpha, beta)
        reg_cd = reg_CD(alpha, beta)

        # Calculate coefficients using dot product with k coefficients
        cl_val = float(np.dot(kCL, reg_cl).item())
        cd_val = float(np.dot(kCD, reg_cd).item())

        cl_values.append(cl_val)
        cd_values.append(cd_val)

    cl_values = np.array(cl_values)
    cd_values = np.array(cd_values)

    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Plot CL
    ax1.plot(aoa_deg, cl_values, 'b-', linewidth=2, label='CL')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylabel('CL', fontsize=12)
    ax1.set_title('Aerodynamic Coefficients at v = 12 m/s, β = 0°', fontsize=14)
    ax1.legend(loc='best')
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax1.axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    # Plot CD
    ax2.plot(aoa_deg, cd_values, 'r-', linewidth=2, label='CD')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('Angle of Attack (degrees)', fontsize=12)
    ax2.set_ylabel('CD', fontsize=12)
    ax2.legend(loc='best')
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax2.axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    plt.show()

    return aoa_deg, cl_values, cd_values


def plot_all_aerodynamic_coeffs():
    """
    Plot all aerodynamic coefficients (CL, CD, CY, Cl, Cm, Cn) across AoA from -180° to 180°
    at 12 m/s velocity norm and β = 0°.
    """
    # Set velocity parameters
    v_norm = 12.0  # m/s
    beta = 0.0  # sideslip angle in radians

    # Generate AoA range from -180 to 180 degrees
    aoa_deg = np.linspace(-180, 180, 361)  # 1-degree increments
    aoa_rad = np.deg2rad(aoa_deg)

    # Calculate all coefficients for each AoA
    cl_values, cd_values, cy_values = [], [], []
    cll_values, cm_values, cn_values = [], [], []

    for alpha in aoa_rad:
        # Calculate regression terms for forces
        reg_cl = reg_CL(alpha, beta)
        reg_cd = reg_CD(alpha, beta)
        reg_cy = reg_CY(alpha, beta)

        # Calculate regression terms for moments
        reg_cll = reg_Cll(alpha, beta)
        reg_cm = reg_Cm(alpha, beta)
        reg_cn = reg_Cn(alpha, beta)

        # Calculate force coefficients
        cl_values.append(float(np.dot(kCL, reg_cl).item()))
        cd_values.append(float(np.dot(kCD, reg_cd).item()))
        cy_values.append(float(np.dot(kCY, reg_cy).item()))

        # Calculate moment coefficients
        cll_values.append(float(np.dot(kCLL, reg_cll).item()))
        cm_values.append(float(np.dot(kCm, reg_cm).item()))
        cn_values.append(float(np.dot(kCn, reg_cn).item()))

    # Convert to numpy arrays
    cl_values = np.array(cl_values)
    cd_values = np.array(cd_values)
    cy_values = np.array(cy_values)
    cll_values = np.array(cll_values)
    cm_values = np.array(cm_values)
    cn_values = np.array(cn_values)

    # Create 2x3 subplot layout
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))

    # First row: CL, CD, Cm
    axes[0, 0].plot(aoa_deg, cl_values, 'b-', linewidth=2, label='CL')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_ylabel('CL', fontsize=12)
    axes[0, 0].set_title('Lift Coefficient at v = 12 m/s, β = 0°', fontsize=14)
    axes[0, 0].legend(loc='best')
    axes[0, 0].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[0, 0].axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    axes[0, 1].plot(aoa_deg, cd_values, 'r-', linewidth=2, label='CD')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_ylabel('CD', fontsize=12)
    axes[0, 1].set_title('Drag Coefficient at v = 12 m/s, β = 0°', fontsize=14)
    axes[0, 1].legend(loc='best')
    axes[0, 1].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[0, 1].axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    axes[0, 2].plot(aoa_deg, cm_values, 'c-', linewidth=2, label='Cm')
    axes[0, 2].grid(True, alpha=0.3)
    axes[0, 2].set_ylabel('Cm', fontsize=12)
    axes[0, 2].set_title('Pitching Moment Coefficient at v = 12 m/s, β = 0°', fontsize=14)
    axes[0, 2].legend(loc='best')
    axes[0, 2].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[0, 2].axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    # Second row: Cl, CY, Cn
    axes[1, 0].plot(aoa_deg, cll_values, 'm-', linewidth=2, label='Cl')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_xlabel('Angle of Attack (degrees)', fontsize=12)
    axes[1, 0].set_ylabel('Cl', fontsize=12)
    axes[1, 0].set_title('Rolling Moment Coefficient at v = 12 m/s, β = 0°', fontsize=14)
    axes[1, 0].legend(loc='best')
    axes[1, 0].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[1, 0].axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    axes[1, 1].plot(aoa_deg, cy_values, 'g-', linewidth=2, label='CY')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_xlabel('Angle of Attack (degrees)', fontsize=12)
    axes[1, 1].set_ylabel('CY', fontsize=12)
    axes[1, 1].set_title('Side Force Coefficient at v = 12 m/s, β = 0°', fontsize=14)
    axes[1, 1].legend(loc='best')
    axes[1, 1].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[1, 1].axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    axes[1, 2].plot(aoa_deg, cn_values, 'orange', linewidth=2, label='Cn')
    axes[1, 2].grid(True, alpha=0.3)
    axes[1, 2].set_xlabel('Angle of Attack (degrees)', fontsize=12)
    axes[1, 2].set_ylabel('Cn', fontsize=12)
    axes[1, 2].set_title('Yawing Moment Coefficient at v = 12 m/s, β = 0°', fontsize=14)
    axes[1, 2].legend(loc='best')
    axes[1, 2].axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    axes[1, 2].axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    plt.tight_layout()
    plt.show()

    return {
        'aoa_deg': aoa_deg, 'CL': cl_values, 'CD': cd_values, 'CY': cy_values,
        'Cl': cll_values, 'Cm': cm_values, 'Cn': cn_values
    }


def plot_aerodynamic_mesh(azimuth=-127.5):
    """
    Generate 3D mesh plots of aerodynamic coefficients across angle of attack and sideslip angle.

    Args:
        azimuth (float): Azimuth angle for 3D view in degrees (default: -127.5)
    """
    # Generate ranges for alpha and beta
    alpha_deg = np.linspace(-180, 180, 361)  # 1-degree increments
    beta_deg = np.linspace(-180, 180, 361)   # 1-degree increments

    alpha_rad = np.deg2rad(alpha_deg)
    beta_rad = np.deg2rad(beta_deg)

    # Create meshgrid
    ALPHA, BETA = np.meshgrid(alpha_rad, beta_rad)
    ALPHA_DEG, BETA_DEG = np.meshgrid(alpha_deg, beta_deg)

    # Initialize coefficient matrices
    CL = np.zeros_like(ALPHA)
    CD = np.zeros_like(ALPHA)
    CY = np.zeros_like(ALPHA)
    CLL = np.zeros_like(ALPHA)
    CM = np.zeros_like(ALPHA)
    CN = np.zeros_like(ALPHA)

    # Calculate coefficients for each (alpha, beta) pair
    for i in range(len(beta_rad)):
        for j in range(len(alpha_rad)):
            alpha = ALPHA[i, j]
            beta = BETA[i, j]

            # Calculate force coefficients
            reg_cl = reg_CL(alpha, beta)
            reg_cd = reg_CD(alpha, beta)
            reg_cy = reg_CY(alpha, beta)

            # Calculate moment coefficients
            reg_cll = reg_Cll(alpha, beta)
            reg_cm = reg_Cm(alpha, beta)
            reg_cn = reg_Cn(alpha, beta)

            # Calculate force coefficients
            CL[i, j] = float(np.dot(kCL, reg_cl).item())
            CD[i, j] = float(np.dot(kCD, reg_cd).item())
            CY[i, j] = float(np.dot(kCY, reg_cy).item())

            # Calculate moment coefficients
            CLL[i, j] = float(np.dot(kCLL, reg_cll).item())
            CM[i, j] = float(np.dot(kCm, reg_cm).item())
            CN[i, j] = float(np.dot(kCn, reg_cn).item())

    # Create figure with 3x2 subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle("Aerodynamic Force and Moment Coefficient Wind Tunnel Curves", fontsize=16)

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection='3d')
    ax1.view_init(elev=30, azim=azimuth)
    surf1 = ax1.plot_surface(ALPHA_DEG, BETA_DEG, CL, cmap='viridis', alpha=0.8)
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection='3d')
    ax2.view_init(elev=30, azim=azimuth)
    surf2 = ax2.plot_surface(ALPHA_DEG, BETA_DEG, CY, cmap='viridis', alpha=0.8)
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection='3d')
    ax3.view_init(elev=30, azim=azimuth)
    surf3 = ax3.plot_surface(ALPHA_DEG, BETA_DEG, CD, cmap='viridis', alpha=0.8)
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection='3d')
    ax4.view_init(elev=30, azim=azimuth)
    surf4 = ax4.plot_surface(ALPHA_DEG, BETA_DEG, CLL, cmap='viridis', alpha=0.8)
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection='3d')
    ax5.view_init(elev=30, azim=azimuth)
    surf5 = ax5.plot_surface(ALPHA_DEG, BETA_DEG, CM, cmap='viridis', alpha=0.8)
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection='3d')
    ax6.view_init(elev=30, azim=azimuth)
    surf6 = ax6.plot_surface(ALPHA_DEG, BETA_DEG, CN, cmap='viridis', alpha=0.8)
    ax6.set_xlabel("alpha[deg]")
    ax6.set_ylabel("beta[deg]")
    ax6.set_zlabel("C_n")
    ax6.set_title("Yawing Moment Coefficient C_n")
    plt.colorbar(surf6, ax=ax6, shrink=0.5)

    plt.tight_layout()
    plt.show()

    return {
        'ALPHA_DEG': ALPHA_DEG, 'BETA_DEG': BETA_DEG,
        'CL': CL, 'CD': CD, 'CY': CY,
        'CLL': CLL, 'CM': CM, 'CN': CN
    }


if __name__ == "__main__":
    # Run the comprehensive plotting function when script is executed directly
    plot_all_aerodynamic_coeffs()

