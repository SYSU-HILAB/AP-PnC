# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# Authors:
# Erchao Rong: rongerch@outlook.com
# Zihao Liu: liuzh297@gmail.com
# Junning Liang: gordonliang27@foxmail.com
#
# JAX batched implementation for efficient aerodynamic coefficient calculations
#
import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from .aero_coefficients import get_jax_coefficients

# setting jax printf options for better readability

COEFFICIENTS = get_jax_coefficients()
kCL = COEFFICIENTS.kCL
kCD = COEFFICIENTS.kCD
kCY = COEFFICIENTS.kCY
kCLL = COEFFICIENTS.kCLL
kCm = COEFFICIENTS.kCm
kCn = COEFFICIENTS.kCn


@jax.jit
def logistic_func_single(alpha, alpha1, k1, alpha2, k2):
    """Single logistic function for use with vmap"""
    P1 = 1.0 / (1 + jnp.exp(k1 * (alpha - alpha1)))
    P2 = 1.0 / (1 + jnp.exp(-k2 * (alpha - alpha2)))
    yval = 1 - P1 - P2
    return yval


# Vectorized version using vmap
logistic_func_batch = jax.vmap(
    logistic_func_single, in_axes=(0, None, None, None, None)
)


@jax.jit
def deg2rad_single(deg):
    """Single degree to radian conversion for use with vmap"""
    return deg * (jnp.pi / 180.0)


# Vectorized version using vmap
deg2rad_batch = jax.vmap(deg2rad_single)


def _regression_batch(reg_single, alpha, beta):
    """Apply regression function to scalar, 1D, or 2D inputs."""
    if alpha.ndim == 0:
        return reg_single(alpha, beta)
    if alpha.ndim == 1:
        return jax.vmap(reg_single, in_axes=(0, 0))(alpha, beta)
    if alpha.ndim == 2:
        return jax.vmap(jax.vmap(reg_single, in_axes=(0, 0)), in_axes=(0, 0))(
            alpha, beta
        )
    raise ValueError(
        "Unsupported alpha dimension "
        f"{alpha.ndim} for alpha shape {alpha.shape} and beta shape {beta.shape}"
    )


@jax.jit
def reg_CL_single(alpha, beta):
    """Single regression calculation for CL coefficient"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2
    sa3 = sa**3
    ca3 = ca**3

    comp_B = jnp.stack([cb2, alpha * cb2])

    comp_N = jnp.stack(
        [
            cb2 * ca3,
            ca * sb2,
            ca * cb2 * sa2,
            sa * ca2 * cb2,
            sa * sb2,
            sa3 * cb2,
            jnp.abs(sb) * cb * ca2,
            sa * ca * cb * jnp.abs(sb),
            jnp.abs(sb) * cb * sa2,
        ]
    )

    w_B1 = logistic_func_single(alpha, deg2rad_single(-190), 1, deg2rad_single(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_single(180)) * cb2])
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func_single(
        alpha, deg2rad_single(-170), 1, deg2rad_single(-20), 0.5
    )
    reg_N11 = w_N11 * comp_N

    w_B2 = logistic_func_single(alpha, deg2rad_single(-11), 20, deg2rad_single(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func_single(alpha, deg2rad_single(18), 10, deg2rad_single(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func_single(alpha, deg2rad_single(170), 1, deg2rad_single(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_single(180)) * cb2])
    reg_B3 = w_B3 * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3])
    return reg


# Create flexible batch function that handles 1D and 2D inputs
def reg_CL_batch(alpha, beta):
    """Flexible batch function that works with both 1D and 2D inputs"""
    return _regression_batch(reg_CL_single, alpha, beta)


@jax.jit
def reg_CD_single(alpha, beta):
    """Single regression calculation for CD coefficient"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2
    sa3 = sa**3
    ca3 = ca**3

    comp_B = jnp.stack([cb2, alpha * cb2])

    comp_N = jnp.stack(
        [
            cb2 * ca3,
            ca * sb2,
            ca * cb2 * sa2,
            sa * ca2 * cb2,
            sa * sb2,
            sa3 * cb2,
            jnp.abs(sb) * cb * ca2,
            sa * ca * cb * jnp.abs(sb),
            jnp.abs(sb) * cb * sa2,
        ]
    )

    w_B1 = logistic_func_single(alpha, deg2rad_single(-190), 1, deg2rad_single(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_single(180)) * cb2])
    reg_B1 = w_B1 * comp_B1

    w_N1 = logistic_func_single(
        alpha, deg2rad_single(-170), 1, deg2rad_single(-10), 0.5
    )
    reg_N1 = w_N1 * comp_N

    w_B2 = logistic_func_single(alpha, deg2rad_single(-11), 20, deg2rad_single(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func_single(alpha, deg2rad_single(18), 10, deg2rad_single(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func_single(alpha, deg2rad_single(170), 1, deg2rad_single(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_single(180)) * cb2])
    reg_B3 = w_B3 * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3])
    return reg


# Create flexible batch function that handles 1D and 2D inputs
def reg_CD_batch(alpha, beta):
    """Flexible batch function that works with both 1D and 2D inputs"""
    return _regression_batch(reg_CD_single, alpha, beta)


@jax.jit
def reg_CY_single(alpha, beta):
    """Single regression calculation for CY coefficient"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    signb = beta * jnp.abs(beta)

    comp_N = jnp.stack([signb * sb2, sb * cb * ca, sb * cb * sa])

    w_B1 = logistic_func_single(alpha, deg2rad_single(-190), 1, deg2rad_single(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_single(180)) * cb2])
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func_single(alpha, deg2rad_single(-170), 1, deg2rad_single(-90), 1)
    reg_N11 = w_N11 * comp_N

    w_N12 = logistic_func_single(alpha, deg2rad_single(-90), 1, deg2rad_single(-11), 1)
    reg_N12 = w_N12 * comp_N

    w_N2 = logistic_func_single(alpha, deg2rad_single(20), 1, deg2rad_single(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func_single(alpha, deg2rad_single(170), 1, deg2rad_single(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_single(180)) * cb2])
    reg_B3 = w_B3 * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_N12, reg_N2, reg_B3])
    return reg


# Create flexible batch function that handles 1D and 2D inputs
def reg_CY_batch(alpha, beta):
    """Flexible batch function that works with both 1D and 2D inputs"""
    return _regression_batch(reg_CY_single, alpha, beta)


@jax.jit
def reg_Cll_single(alpha, beta):
    """Single regression calculation for Cl coefficient"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    signb = beta * jnp.abs(beta)

    comp_B = jnp.stack([cb2, alpha * cb2])
    comp_N = jnp.stack([signb * sb2, sb * cb * ca, sb * cb * sa])

    w_B1 = logistic_func_single(alpha, deg2rad_single(-190), 1, deg2rad_single(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_single(180)) * cb2])
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func_single(alpha, deg2rad_single(-170), 1, deg2rad_single(-90), 5)
    reg_N11 = w_N11 * comp_N

    w_N12 = logistic_func_single(
        alpha, deg2rad_single(-90), 5, deg2rad_single(-20), 0.5
    )
    reg_N12 = w_N12 * comp_N

    w_B2 = logistic_func_single(alpha, deg2rad_single(-11), 20, deg2rad_single(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N22 = logistic_func_single(alpha, deg2rad_single(18), 10, deg2rad_single(90), 5)
    reg_N22 = w_N22 * comp_N

    w_N2 = logistic_func_single(alpha, deg2rad_single(90), 5, deg2rad_single(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func_single(alpha, deg2rad_single(170), 1, deg2rad_single(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_single(180)) * cb2])
    reg_B3 = w_B3 * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_N12, reg_B2, reg_N2, reg_N22, reg_B3])
    return reg


# Create flexible batch function that handles 1D and 2D inputs
def reg_Cll_batch(alpha, beta):
    """Flexible batch function that works with both 1D and 2D inputs"""
    return _regression_batch(reg_Cll_single, alpha, beta)


@jax.jit
def reg_Cm_single(alpha, beta):
    """Single regression calculation for Cm coefficient"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2

    comp_B = jnp.stack([cb2, alpha * cb2])
    comp_N = jnp.stack(
        [
            cb2 * ca2,
            sb2,
            cb2 * sa2,
            jnp.abs(sb) * cb * ca,
            sa * ca * cb2,
            jnp.abs(sb) * cb * sa,
        ]
    )

    w_B1 = logistic_func_single(alpha, deg2rad_single(-190), 1, deg2rad_single(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_single(180)) * cb2])
    reg_B1 = w_B1 * comp_B1

    w_N11 = logistic_func_single(
        alpha, deg2rad_single(-170), 1, deg2rad_single(-20), 0.5
    )
    reg_N11 = w_N11 * comp_N

    w_B2 = logistic_func_single(alpha, deg2rad_single(-11), 20, deg2rad_single(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func_single(alpha, deg2rad_single(18), 10, deg2rad_single(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func_single(alpha, deg2rad_single(170), 1, deg2rad_single(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_single(180)) * cb2])
    reg_B3 = w_B3 * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3])
    return reg


# Create flexible batch function that handles 1D and 2D inputs
def reg_Cm_batch(alpha, beta):
    """Flexible batch function that works with both 1D and 2D inputs"""
    return _regression_batch(reg_Cm_single, alpha, beta)


@jax.jit
def reg_Cn_single(alpha, beta):
    """Single regression calculation for Cn coefficient"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    signb = beta * jnp.abs(beta)

    comp_B = jnp.stack([cb2, alpha * cb2])
    comp_N = jnp.stack([signb * sb2, sb * cb * ca, sb * cb * sa])

    w_B1 = logistic_func_single(alpha, deg2rad_single(-190), 1, deg2rad_single(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_single(180)) * cb2])
    reg_B1 = w_B1 * comp_B1

    w_N1 = logistic_func_single(alpha, deg2rad_single(-170), 1, deg2rad_single(-10), 10)
    reg_N1 = w_N1 * comp_N

    w_B2 = logistic_func_single(alpha, deg2rad_single(-10), 10, deg2rad_single(18), 10)
    reg_B2 = w_B2 * comp_B

    w_N2 = logistic_func_single(alpha, deg2rad_single(0), 0.5, deg2rad_single(170), 1)
    reg_N2 = w_N2 * comp_N

    w_B3 = logistic_func_single(alpha, deg2rad_single(170), 1, deg2rad_single(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_single(180)) * cb2])
    reg_B3 = w_B3 * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3])
    return reg


# Create flexible batch function that handles 1D and 2D inputs
def reg_Cn_batch(alpha, beta):
    """Flexible batch function that works with both 1D and 2D inputs"""
    return _regression_batch(reg_Cn_single, alpha, beta)


@jax.jit
def compute_aerodynamic_coeffs_jax(ALPHA, BETA):
    """
    Compute all aerodynamic coefficients using JAX with JIT compilation.
    Returns all coefficients without plotting.
    """
    # Calculate coefficients using vmap-based batch functions
    reg_cl = reg_CL_batch(ALPHA, BETA)
    reg_cd = reg_CD_batch(ALPHA, BETA)
    reg_cy = reg_CY_batch(ALPHA, BETA)
    reg_cll = reg_Cll_batch(ALPHA, BETA)
    reg_cm = reg_Cm_batch(ALPHA, BETA)
    reg_cn = reg_Cn_batch(ALPHA, BETA)

    # Coefficient calculation using vmap - no manual broadcasting needed
    CL = jnp.sum(reg_cl * kCL, axis=-1)
    CD = jnp.sum(reg_cd * kCD, axis=-1)
    CY = jnp.sum(reg_cy * kCY, axis=-1)
    CLL = jnp.sum(reg_cll * kCLL, axis=-1)
    CM = jnp.sum(reg_cm * kCm, axis=-1)
    CN = jnp.sum(reg_cn * kCn, axis=-1)

    return CL, CD, CY, CLL, CM, CN


def compute_angles_single(V_b):
    # V_b is expected to be shape (3,) -> [u, v, w]
    vbx = V_b[0]
    vby = V_b[1]
    vbz = V_b[2]

    # Calculation
    v_norm = jnp.sqrt(vbx**2 + vby**2 + vbz**2) + 1e-8
    v_squared = v_norm**2
    alpha = jnp.arctan2(vbz, vbx)
    beta = jnp.arcsin(vby / v_norm)

    return alpha, beta, v_norm, v_squared


def compute_angles_batch(V_b):
    if V_b.ndim == 1:  # Single vector
        return compute_angles_single(V_b)
    elif V_b.ndim == 2:  # Batch of vectors
        return jax.vmap(compute_angles_single, in_axes=(0,))(V_b)
    else:
        raise ValueError(f"Unsupported V_b dimension: {V_b.ndim}")


def trans_stab_to_body_single(alpha, vec):
    """
    Docstring for trans_stab_to_body

    :param vec: Array of shape (..., 3) representing vectors in stability frame
    :param alpha: Angle of attack in radians
    :return: Transformed vectors in body frame of shape (..., 3)
    """
    # Stability to body frame transformation matrix
    T_sb = jnp.array(
        [
            [jnp.cos(alpha), 0.0, -jnp.sin(alpha)],
            [0.0, 1.0, 0.0],
            [jnp.sin(alpha), 0.0, jnp.cos(alpha)],
        ],
        dtype=jnp.float32,
    )
    return jnp.einsum("ij,...j->...i", T_sb, vec)


def trans_stab_to_body_batch(ALPHA, VEC):
    """
    Batch version
    :param ALPHA: Array of shape (...) representing angles of attack in radians
    :param VEC: Array of shape (..., 3) representing vectors in stability frame
    :return: Transformed vectors in body frame of shape (..., 3)
    """
    if ALPHA.ndim == 0:  # Single alpha
        return trans_stab_to_body_single(ALPHA, VEC)
    elif ALPHA.ndim == 1:  # 1D arrays
        return jax.vmap(trans_stab_to_body_single, in_axes=(0, 0))(ALPHA, VEC)
    else:  # 2D arrays
        raise ValueError(f"Unsupported ALPHA dimension: {ALPHA.ndim}")


def aerodynamics_B_vb(V_bs):
    """
    Docstring for aerodynamics_vb

    :param V_bs: Array of shape (..., 3) representing body-frame velocities
    :return: FORCES and MOMENTS arrays of shape (..., 3)
    """
    ALPHA, BETA, V_NORM, V_SQUARED = compute_angles_batch(V_bs)
    CL, CD, CY, CLL, CM, CN = compute_aerodynamic_coeffs_jax(ALPHA, BETA)
    rho = 1.185  # kg/m^3
    FORCES_DYL = jnp.stack(
        [
            -0.5 * rho * V_SQUARED * CD,
            0.5 * rho * V_SQUARED * CY,
            -0.5 * rho * V_SQUARED * CL,
        ],
        axis=-1,
    )

    FORCES_B = trans_stab_to_body_batch(ALPHA, FORCES_DYL)

    MOMENTS_B = jnp.stack(
        [
            0.5 * rho * V_SQUARED * CLL,
            0.5 * rho * V_SQUARED * CM,
            0.5 * rho * V_SQUARED * CN,
        ],
        axis=-1,
    )
    return FORCES_B, MOMENTS_B, ALPHA, BETA


def aerodynamics_DYL_vb(V_bs):
    """
    Docstring for aerodynamics_vb

    :param V_bs: Array of shape (..., 3) representing body-frame velocities
    :return: FORCES and MOMENTS arrays of shape (..., 3)
    """
    ALPHA, BETA, V_NORM, V_SQUARED = compute_angles_batch(V_bs)
    CL, CD, CY, CLL, CM, CN = compute_aerodynamic_coeffs_jax(ALPHA, BETA)
    rho = 1.185  # kg/m^3
    FORCES = jnp.stack(
        [
            -0.5 * rho * V_SQUARED * CD,
            0.5 * rho * V_SQUARED * CY,
            -0.5 * rho * V_SQUARED * CL,
        ],
        axis=-1,
    )
    MOMENTS = jnp.stack(
        [
            0.5 * rho * V_SQUARED * CLL,
            0.5 * rho * V_SQUARED * CM,
            0.5 * rho * V_SQUARED * CN,
        ],
        axis=-1,
    )
    return FORCES, MOMENTS, ALPHA, BETA

def plot_aerodynamic_mesh_jax(azimuth=125.5):
    """
    Generate 3D mesh plots of aerodynamic coefficients using batched JAX calculations.
    Much faster than the original numpy version.

    Args:
        azimuth (float): Azimuth angle for 3D view in degrees (default: -127.5)
    """
    print("Setting up mesh grid...")

    # Check if GPU is available in JAX
    devices = jax.devices()
    device = devices[0] if devices else jax.devices("cpu")[0]
    print(f"Using device: {device}")
    if hasattr(device, "platform"):
        print(f"Platform: {device.platform}")

    # Generate ranges for alpha and beta
    alpha_deg = jnp.linspace(-180, 180, 3601)  # 1-degree increments
    beta_deg = jnp.linspace(-90, 90, 1801)  # 1-degree increments

    alpha_rad = deg2rad_batch(alpha_deg)
    beta_rad = deg2rad_batch(beta_deg)

    # Create meshgrid
    ALPHA_DEG, BETA_DEG = jnp.meshgrid(alpha_deg, beta_deg, indexing="ij")
    ALPHA, BETA = jnp.meshgrid(alpha_rad, beta_rad, indexing="ij")

    print("Computing aerodynamic coefficients in batches...")

    # Calculate coefficients in batches
    reg_cl = reg_CL_batch(ALPHA, BETA)
    reg_cd = reg_CD_batch(ALPHA, BETA)
    reg_cy = reg_CY_batch(ALPHA, BETA)
    reg_cll = reg_Cll_batch(ALPHA, BETA)
    reg_cm = reg_Cm_batch(ALPHA, BETA)
    reg_cn = reg_Cn_batch(ALPHA, BETA)

    # Coefficient calculation using vmap - no manual broadcasting needed
    CL = jnp.sum(reg_cl * kCL, axis=-1)
    CD = jnp.sum(reg_cd * kCD, axis=-1)
    CY = jnp.sum(reg_cy * kCY, axis=-1)
    CLL = jnp.sum(reg_cll * kCLL, axis=-1)
    CM = jnp.sum(reg_cm * kCm, axis=-1)
    CN = jnp.sum(reg_cn * kCn, axis=-1)

    # Convert to numpy for plotting
    print("Converting results to numpy for plotting...")
    ALPHA_DEG_np = np.array(ALPHA_DEG)
    BETA_DEG_np = np.array(BETA_DEG)
    CL_np = np.array(CL)
    CD_np = np.array(CD)
    CY_np = np.array(CY)
    CLL_np = np.array(CLL)
    CM_np = np.array(CM)
    CN_np = np.array(CN)

    print("Creating plots...")
    # Create figure with 3x2 subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle(
        "Aerodynamic Force and Moment Coefficient Wind Tunnel Curves", fontsize=16
    )

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection="3d")
    ax1.view_init(elev=10, azim=azimuth)
    surf1 = ax1.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CL_np, cmap="viridis", alpha=0.8
    )
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection="3d")
    ax2.view_init(elev=10, azim=azimuth)
    surf2 = ax2.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CY_np, cmap="viridis", alpha=0.8
    )
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection="3d")
    ax3.view_init(elev=10, azim=azimuth)
    surf3 = ax3.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CD_np, cmap="viridis", alpha=0.8
    )
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection="3d")
    ax4.view_init(elev=10, azim=azimuth)
    surf4 = ax4.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CLL_np, cmap="viridis", alpha=0.8
    )
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection="3d")
    ax5.view_init(elev=10, azim=azimuth)
    surf5 = ax5.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CM_np, cmap="viridis", alpha=0.8
    )
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection="3d")
    ax6.view_init(elev=10, azim=azimuth)
    surf6 = ax6.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CN_np, cmap="viridis", alpha=0.8
    )
    ax6.set_xlabel("alpha[deg]")
    ax6.set_ylabel("beta[deg]")
    ax6.set_zlabel("C_n")
    ax6.set_title("Yawing Moment Coefficient C_n")
    plt.colorbar(surf6, ax=ax6, shrink=0.5)

    plt.tight_layout()
    plt.show()

    print("JAX batched mesh plotting completed successfully!")

    return {
        "ALPHA_DEG": ALPHA_DEG_np,
        "BETA_DEG": BETA_DEG_np,
        "CL": CL_np,
        "CD": CD_np,
        "CY": CY_np,
        "CLL": CLL_np,
        "CM": CM_np,
        "CN": CN_np,
    }


def plot_cl_cd_cm_vs_alpha_coordinated():
    """
    Plot CL, CD, CM coefficients versus alpha under coordinated flight assumption (beta = 0).
    Creates one row with three subplots for each coefficient.
    """
    import matplotlib.pyplot as plt

    print("Computing aerodynamic coefficients under coordinated flight (beta = 0)...")

    # Generate alpha range (angle of attack)
    alpha_deg = jnp.linspace(
        -30, 120, 1501
    )  # -30 to 120 degrees, 0.1 degree increments
    alpha_rad = deg2rad_batch(alpha_deg)

    # Coordinated flight assumption: beta = 0 (sideslip angle)
    beta = jnp.zeros_like(alpha_rad)  # Beta = 0 for all alpha values

    # Compute aerodynamic coefficients
    CL, CD, CY, CLL, CM, CN = compute_aerodynamic_coeffs_jax(alpha_rad, beta)

    # Convert to numpy for plotting
    alpha_deg_np = np.array(alpha_deg)
    CL_np = np.array(CL)
    CD_np = np.array(CD)
    CM_np = np.array(CM)

    # Create the plot with one row and three subplots
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(
        "Aerodynamic Coefficients vs Angle of Attack (Coordinated Flight: β = 0°)",
        fontsize=16,
    )

    # CL subplot
    ax1.plot(alpha_deg_np, CL_np, "b-", linewidth=2)
    ax1.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax1.set_ylabel("CL (Lift Coefficient)", fontsize=12)
    ax1.set_title("Lift Coefficient CL", fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax1.axvline(x=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax1.set_xlim([-30, 120])

    # CD subplot
    ax2.plot(alpha_deg_np, CD_np, "r-", linewidth=2)
    ax2.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax2.set_ylabel("CD (Drag Coefficient)", fontsize=12)
    ax2.set_title("Drag Coefficient CD", fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax2.axvline(x=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax2.set_xlim([-30, 120])

    # CM subplot
    ax3.plot(alpha_deg_np, CM_np, "g-", linewidth=2)
    ax3.set_xlabel("Angle of Attack α [degrees]", fontsize=12)
    ax3.set_ylabel("CM (Pitching Moment Coefficient)", fontsize=12)
    ax3.set_title("Pitching Moment Coefficient CM", fontsize=14)
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax3.axvline(x=0, color="k", linestyle="--", alpha=0.3, linewidth=1)
    ax3.set_xlim([-30, 120])

    plt.tight_layout()
    plt.show()

    print("Plot completed!")

    return {"alpha_deg": alpha_deg_np, "CL": CL_np, "CD": CD_np, "CM": CM_np}


def generate_cl_cd_cm_coordinated_data(
    filename="coordinated_aero_data_jax", cache_dir=None
):
    """
    Generate CL, CD, CM coefficients data under coordinated flight assumption (beta = 0).
    Saves data to .npy file for later plotting.

    Args:
        filename (str): Name for the data file (without extension)
        cache_dir (str, optional): Custom cache directory

    Returns:
        str: Path to the saved data file
    """
    print("Generating aerodynamic coefficients under coordinated flight (beta = 0)...")

    # Import data utilities
    from ...plotting.data_utils import save_coordinated_data

    # Generate alpha range (angle of attack)
    alpha_deg = jnp.linspace(
        -30, 120, 1501
    )  # -30 to 120 degrees, 0.1 degree increments
    alpha_rad = deg2rad_batch(alpha_deg)

    # Coordinated flight assumption: beta = 0 (sideslip angle)
    beta = jnp.zeros_like(alpha_rad)  # Beta = 0 for all alpha values

    # Compute aerodynamic coefficients
    CL, CD, CY, CLL, CM, CN = compute_aerodynamic_coeffs_jax(alpha_rad, beta)

    # Convert to numpy for saving
    alpha_deg_np = np.array(alpha_deg)
    CL_np = np.array(CL)
    CD_np = np.array(CD)
    CM_np = np.array(CM)

    # Save data using data utilities
    save_coordinated_data(
        alpha_deg_np, CL_np, CD_np, CM_np, filename=filename, cache_dir=cache_dir
    )

    print("Coordinated flight data generation completed!")
    print(
        f"Data shape: alpha={alpha_deg_np.shape}, CL={CL_np.shape}, CD={CD_np.shape}, CM={CM_np.shape}"
    )

    # Return the data dictionary for immediate use
    return {"alpha_deg": alpha_deg_np, "CL": CL_np, "CD": CD_np, "CM": CM_np}


def generate_mesh_data_jax(
    filename="mesh_aero_data_jax", cache_dir=None, mesh_size=361
):
    """
    Generate 3D mesh data for aerodynamic coefficients using JAX.
    Saves data to .npy file for later plotting.

    Args:
        filename (str): Name for the data file (without extension)
        cache_dir (str, optional): Custom cache directory
        mesh_size (int): Size of the mesh grid (mesh_size x mesh_size points)

    Returns:
        str: Path to the saved data file
    """
    print("Generating JAX 3D mesh data...")

    # Import data utilities
    from ...plotting.data_utils import save_mesh_data

    # Check if GPU is available in JAX
    devices = jax.devices()
    device = devices[0] if devices else jax.devices("cpu")[0]
    print(f"Using device: {device}")
    if hasattr(device, "platform"):
        print(f"Platform: {device.platform}")

    # Generate ranges for alpha and beta
    alpha_deg = jnp.linspace(-180, 180, mesh_size)
    beta_deg = jnp.linspace(-90, 90, mesh_size)

    alpha_rad = deg2rad_batch(alpha_deg)
    beta_rad = deg2rad_batch(beta_deg)

    # Create meshgrid
    ALPHA_DEG, BETA_DEG = jnp.meshgrid(alpha_deg, beta_deg, indexing="ij")
    ALPHA, BETA = jnp.meshgrid(alpha_rad, beta_rad, indexing="ij")

    print("Computing aerodynamic coefficients in batches...")

    # Calculate coefficients in batches
    CL, CD, CY, CLL, CM, CN = compute_aerodynamic_coeffs_jax(ALPHA, BETA)

    # Convert to numpy for saving
    ALPHA_DEG_np = np.array(ALPHA_DEG)
    BETA_DEG_np = np.array(BETA_DEG)
    CL_np = np.array(CL)
    CD_np = np.array(CD)
    CY_np = np.array(CY)
    CLL_np = np.array(CLL)
    CM_np = np.array(CM)
    CN_np = np.array(CN)

    # Save data using data utilities
    save_mesh_data(
        ALPHA_DEG_np,
        BETA_DEG_np,
        CL_np,
        CD_np,
        CY_np,
        CLL_np,
        CM_np,
        CN_np,
        filename=filename,
        cache_dir=cache_dir,
        backend="jax",
    )

    print("JAX mesh data generation completed!")
    print(
        f"Data shape: ALPHA={ALPHA_DEG_np.shape}, BETA={BETA_DEG_np.shape}, CL={CL_np.shape}"
    )

    # Return the data dictionary for immediate use
    return {
        "ALPHA_DEG": ALPHA_DEG_np,
        "BETA_DEG": BETA_DEG_np,
        "CL": CL_np,
        "CD": CD_np,
        "CY": CY_np,
        "CLL": CLL_np,
        "CM": CM_np,
        "CN": CN_np,
    }
