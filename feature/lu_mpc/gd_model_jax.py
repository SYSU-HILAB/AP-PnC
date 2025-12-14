# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@outlook.com
#
# JAX batched implementation for efficient aerodynamic coefficient calculations
#
import math
import time
import numpy as np
import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Coefficient arrays - same as original numpy version
kCL = jnp.array([
    -2.8465, 23.1516, -0.7335, 0.1407, 0.0930, 3.8276, -0.0006, 3.1951, -0.0289, 0.6425, 0.0164, 0.0031, 0.6174, -0.1286, 0.0342, -0.5109, 0.0982, -0.0006, 0.1985, -0.0070, 0.0410, 0.0037, -4.3496, 14.1041
], dtype=jnp.float32)

kCD = jnp.array([
    -0.5908, -1.8161, -0.2045, -0.0036, -0.2861, -0.8862, -0.0590, -0.9215, 0.0130, -0.0035, 0.4958, -0.0032, 0.0762, 0.1533, -0.0009, 0.1891, 0.0159, 0.0172, 0.2510, 0.0032, -0.1227, 0.1740, 0.7917, -4.4927
], dtype=jnp.float32)

kCY = jnp.array([
    -0.0678, 0.1151, -0.1024, 0.6002, 0.5367, -0.0922, -0.8523, -0.5220, -0.1006, 0.2266, -0.0345, -0.0768, 0.0748
], dtype=jnp.float32)

kCLL = jnp.array([
    -0.0240, 0.0172, -0.0219, -0.0085, -0.2163, -0.0143, -0.0152, -0.1874, -0.0021, -0.0065, -0.0110, 0.0029, -0.0184, -0.0028, 0.0203, -0.0158, -0.0245, -0.0068
], dtype=jnp.float32)

kCm = jnp.array([
    -1.9963, -2.5059, 0.5987, 0.0120, 0.7977, 0.0543, 0.6310, -0.1122, -0.0798, -0.2177, -0.1059, 0.0029, -0.1388, 0.0132, -0.2605, 0.0015, -0.5789, 0.7902
], dtype=jnp.float32)

kCn = jnp.array([
    0.0681, -0.0338, 0.0054, 0.0028, -0.0052, 0.0034, -0.0048, 0.0217, 0.0111, 0.0121, 0.0685, 0.0240
], dtype=jnp.float32)


@jax.jit
def logistic_func_batch(alpha, alpha1, k1, alpha2, k2):
    """Batched logistic function with JIT compilation"""
    # For JIT compatibility, we handle invalid parameters by returning zeros
    # instead of using print statements and returning None
    valid_params = (k1 >= 0) & (k2 >= 0) & (alpha1 < alpha2)

    P1 = 1.0 / (1 + jnp.exp(k1 * (alpha - alpha1)))
    P2 = 1.0 / (1 + jnp.exp(-k2 * (alpha - alpha2)))
    yval = 1 - P1 - P2

    # Return zeros for invalid parameters, yval for valid ones
    return jnp.where(valid_params, yval, jnp.zeros_like(alpha))


@jax.jit
def deg2rad_batch(deg):
    """Batched degree to radian conversion with JIT compilation"""
    return deg * (jnp.pi / 180.0)


@jax.jit
def reg_CL_batch(alpha, beta):
    """Batched regression calculation for CL coefficient with JIT compilation"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sa2 = sa ** 2
    ca2 = ca ** 2
    sb2 = sb ** 2
    cb2 = cb ** 2
    sa3 = sa ** 3
    ca3 = ca ** 3

    comp_B = jnp.stack([cb2, alpha * cb2], axis=-1)

    comp_N = jnp.stack([
        cb2 * ca3,
        ca * sb2,
        ca * cb2 * sa2,
        sa * ca2 * cb2,
        sa * sb2,
        sa3 * cb2,
        jnp.abs(sb) * cb * ca2,
        sa * ca * cb * jnp.abs(sb),
        jnp.abs(sb) * cb * sa2,
    ], axis=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], axis=-1)
    reg_B1 = w_B1[..., None] * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-20), 0.5)
    reg_N11 = w_N11[..., None] * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2[..., None] * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(170), 1)
    reg_N2 = w_N2[..., None] * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], axis=-1)
    reg_B3 = w_B3[..., None] * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3], axis=-1)
    return reg


@jax.jit
def reg_CD_batch(alpha, beta):
    """Batched regression calculation for CD coefficient with JIT compilation"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sa2 = sa ** 2
    ca2 = ca ** 2
    sb2 = sb ** 2
    cb2 = cb ** 2
    sa3 = sa ** 3
    ca3 = ca ** 3

    comp_B = jnp.stack([cb2, alpha * cb2], axis=-1)

    comp_N = jnp.stack([
        cb2 * ca3,
        ca * sb2,
        ca * cb2 * sa2,
        sa * ca2 * cb2,
        sa * sb2,
        sa3 * cb2,
        jnp.abs(sb) * cb * ca2,
        sa * ca * cb * jnp.abs(sb),
        jnp.abs(sb) * cb * sa2,
    ], axis=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], axis=-1)
    reg_B1 = w_B1[..., None] * comp_B1

    w_N1 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-10), 0.5)
    reg_N1 = w_N1[..., None] * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2[..., None] * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(170), 1)
    reg_N2 = w_N2[..., None] * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], axis=-1)
    reg_B3 = w_B3[..., None] * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3], axis=-1)
    return reg


@jax.jit
def reg_CY_batch(alpha, beta):
    """Batched regression calculation for CY coefficient with JIT compilation"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sb2 = sb ** 2
    cb2 = cb ** 2
    signb = beta * jnp.abs(beta)

    comp_N = jnp.stack([signb * sb2, sb * cb * ca, sb * cb * sa], axis=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], axis=-1)
    reg_B1 = w_B1[..., None] * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-90), 1)
    reg_N11 = w_N11[..., None] * comp_N

    w_N12 = logistic_func_batch(alpha, deg2rad_batch(-90), 1, deg2rad_batch(-11), 1)
    reg_N12 = w_N12[..., None] * comp_N

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(20), 1, deg2rad_batch(170), 1)
    reg_N2 = w_N2[..., None] * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], axis=-1)
    reg_B3 = w_B3[..., None] * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_N12, reg_N2, reg_B3], axis=-1)
    return reg


@jax.jit
def reg_Cll_batch(alpha, beta):
    """Batched regression calculation for Cl coefficient with JIT compilation"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sb2 = sb ** 2
    cb2 = cb ** 2
    signb = beta * jnp.abs(beta)

    comp_B = jnp.stack([cb2, alpha * cb2], axis=-1)
    comp_N = jnp.stack([signb * sb2, sb * cb * ca, sb * cb * sa], axis=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], axis=-1)
    reg_B1 = w_B1[..., None] * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-90), 5)
    reg_N11 = w_N11[..., None] * comp_N

    w_N12 = logistic_func_batch(alpha, deg2rad_batch(-90), 5, deg2rad_batch(-20), 0.5)
    reg_N12 = w_N12[..., None] * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2[..., None] * comp_B

    w_N22 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(90), 5)
    reg_N22 = w_N22[..., None] * comp_N

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(90), 5, deg2rad_batch(170), 1)
    reg_N2 = w_N2[..., None] * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], axis=-1)
    reg_B3 = w_B3[..., None] * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_N12, reg_B2, reg_N2, reg_N22, reg_B3], axis=-1)
    return reg


@jax.jit
def reg_Cm_batch(alpha, beta):
    """Batched regression calculation for Cm coefficient with JIT compilation"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sa2 = sa ** 2
    ca2 = ca ** 2
    sb2 = sb ** 2
    cb2 = cb ** 2

    comp_B = jnp.stack([cb2, alpha * cb2], axis=-1)
    comp_N = jnp.stack([cb2 * ca2, sb2, cb2 * sa2, jnp.abs(sb) * cb * ca, sa * ca * cb2, jnp.abs(sb) * cb * sa], axis=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], axis=-1)
    reg_B1 = w_B1[..., None] * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-20), 0.5)
    reg_N11 = w_N11[..., None] * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2[..., None] * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(170), 1)
    reg_N2 = w_N2[..., None] * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], axis=-1)
    reg_B3 = w_B3[..., None] * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3], axis=-1)
    return reg


@jax.jit
def reg_Cn_batch(alpha, beta):
    """Batched regression calculation for Cn coefficient with JIT compilation"""
    sa = jnp.sin(alpha)
    ca = jnp.cos(alpha)
    sb = jnp.sin(beta)
    cb = jnp.cos(beta)
    sb2 = sb ** 2
    cb2 = cb ** 2
    signb = beta * jnp.abs(beta)

    comp_B = jnp.stack([cb2, alpha * cb2], axis=-1)
    comp_N = jnp.stack([signb * sb2, sb * cb * ca, sb * cb * sa], axis=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = jnp.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], axis=-1)
    reg_B1 = w_B1[..., None] * comp_B1

    w_N1 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-10), 10)
    reg_N1 = w_N1[..., None] * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-10), 10, deg2rad_batch(18), 10)
    reg_B2 = w_B2[..., None] * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(0), 0.5, deg2rad_batch(170), 1)
    reg_N2 = w_N2[..., None] * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = jnp.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], axis=-1)
    reg_B3 = w_B3[..., None] * comp_B3

    reg = jnp.concatenate([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3], axis=-1)
    return reg


@jax.jit
def compute_aerodynamic_coeffs_jax(ALPHA, BETA):
    """
    Compute all aerodynamic coefficients using JAX with JIT compilation - for timing purposes.
    Returns all coefficients without plotting.
    """
    # Calculate coefficients in batches
    reg_cl = reg_CL_batch(ALPHA, BETA)
    reg_cd = reg_CD_batch(ALPHA, BETA)
    reg_cy = reg_CY_batch(ALPHA, BETA)
    reg_cll = reg_Cll_batch(ALPHA, BETA)
    reg_cm = reg_Cm_batch(ALPHA, BETA)
    reg_cn = reg_Cn_batch(ALPHA, BETA)

    # Batched coefficient calculation using multiplication
    CL = jnp.sum(reg_cl * kCL[None, None, :], axis=-1)
    CD = jnp.sum(reg_cd * kCD[None, None, :], axis=-1)
    CY = jnp.sum(reg_cy * kCY[None, None, :], axis=-1)
    CLL = jnp.sum(reg_cll * kCLL[None, None, :], axis=-1)
    CM = jnp.sum(reg_cm * kCm[None, None, :], axis=-1)
    CN = jnp.sum(reg_cn * kCn[None, None, :], axis=-1)

    return CL, CD, CY, CLL, CM, CN


def plot_aerodynamic_mesh_jax():
    """
    Generate 3D mesh plots of aerodynamic coefficients using batched JAX calculations.
    Much faster than the original numpy version.
    """
    print("Setting up mesh grid...")

    # Check if GPU is available in JAX
    devices = jax.devices()
    device = devices[0] if devices else jax.devices('cpu')[0]
    print(f"Using device: {device}")
    if hasattr(device, 'platform'):
        print(f"Platform: {device.platform}")

    # Generate ranges for alpha and beta
    alpha_deg = jnp.linspace(-180, 180, 361)  # 1-degree increments
    beta_deg = jnp.linspace(-180, 180, 361)   # 1-degree increments

    alpha_rad = deg2rad_batch(alpha_deg)
    beta_rad = deg2rad_batch(beta_deg)

    # Create meshgrid
    ALPHA_DEG, BETA_DEG = jnp.meshgrid(alpha_deg, beta_deg, indexing='ij')
    ALPHA, BETA = jnp.meshgrid(alpha_rad, beta_rad, indexing='ij')

    print("Computing aerodynamic coefficients in batches...")

    # Calculate coefficients in batches
    reg_cl = reg_CL_batch(ALPHA, BETA)
    reg_cd = reg_CD_batch(ALPHA, BETA)
    reg_cy = reg_CY_batch(ALPHA, BETA)
    reg_cll = reg_Cll_batch(ALPHA, BETA)
    reg_cm = reg_Cm_batch(ALPHA, BETA)
    reg_cn = reg_Cn_batch(ALPHA, BETA)

    # Batched coefficient calculation using multiplication
    CL = jnp.sum(reg_cl * kCL[None, None, :], axis=-1)
    CD = jnp.sum(reg_cd * kCD[None, None, :], axis=-1)
    CY = jnp.sum(reg_cy * kCY[None, None, :], axis=-1)
    CLL = jnp.sum(reg_cll * kCLL[None, None, :], axis=-1)
    CM = jnp.sum(reg_cm * kCm[None, None, :], axis=-1)
    CN = jnp.sum(reg_cn * kCn[None, None, :], axis=-1)

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
    fig.suptitle("Aerodynamic Force and Moment Coefficient Wind Tunnel Curves", fontsize=16)

    # CL subplot
    ax1 = fig.add_subplot(3, 2, 1, projection='3d')
    surf1 = ax1.plot_surface(ALPHA_DEG_np, BETA_DEG_np, CL_np, cmap='viridis', alpha=0.8)
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient CL")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # CY subplot
    ax2 = fig.add_subplot(3, 2, 3, projection='3d')
    surf2 = ax2.plot_surface(ALPHA_DEG_np, BETA_DEG_np, CY_np, cmap='viridis', alpha=0.8)
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_Y")
    ax2.set_title("Side Force Coefficient CY")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # CD subplot
    ax3 = fig.add_subplot(3, 2, 5, projection='3d')
    surf3 = ax3.plot_surface(ALPHA_DEG_np, BETA_DEG_np, CD_np, cmap='viridis', alpha=0.8)
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient CD")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # CLL subplot
    ax4 = fig.add_subplot(3, 2, 2, projection='3d')
    surf4 = ax4.plot_surface(ALPHA_DEG_np, BETA_DEG_np, CLL_np, cmap='viridis', alpha=0.8)
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # CM subplot
    ax5 = fig.add_subplot(3, 2, 4, projection='3d')
    surf5 = ax5.plot_surface(ALPHA_DEG_np, BETA_DEG_np, CM_np, cmap='viridis', alpha=0.8)
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_m")
    ax5.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # CN subplot
    ax6 = fig.add_subplot(3, 2, 6, projection='3d')
    surf6 = ax6.plot_surface(ALPHA_DEG_np, BETA_DEG_np, CN_np, cmap='viridis', alpha=0.8)
    ax6.set_xlabel("alpha[deg]")
    ax6.set_ylabel("beta[deg]")
    ax6.set_zlabel("C_n")
    ax6.set_title("Yawing Moment Coefficient C_n")
    plt.colorbar(surf6, ax=ax6, shrink=0.5)

    plt.tight_layout()
    plt.show()

    print("JAX batched mesh plotting completed successfully!")

    return {
        'ALPHA_DEG': ALPHA_DEG_np, 'BETA_DEG': BETA_DEG_np,
        'CL': CL_np, 'CD': CD_np, 'CY': CY_np,
        'CLL': CLL_np, 'CM': CM_np, 'CN': CN_np
    }


def benchmark_jax_timing(mesh_size=361, num_runs=10):
    """
    Benchmark JAX computation time for aerodynamic coefficient calculations.
    """
    print(f"\n=== JAX Performance Benchmark ===")
    print(f"Mesh size: {mesh_size}x{mesh_size} points")
    print(f"Number of runs: {num_runs}")

    # Check device
    devices = jax.devices()
    device = devices[0] if devices else jax.devices('cpu')[0]
    print(f"Using device: {device}")
    if hasattr(device, 'platform'):
        print(f"Platform: {device.platform}")

    # Generate ranges for alpha and beta
    alpha_deg = jnp.linspace(-180, 180, mesh_size)
    beta_deg = jnp.linspace(-180, 180, mesh_size)

    alpha_rad = deg2rad_batch(alpha_deg)
    beta_rad = deg2rad_batch(beta_deg)

    # Create meshgrid
    ALPHA, BETA = jnp.meshgrid(alpha_rad, beta_rad, indexing='ij')

    # Warmup run
    print("Performing warmup run...")
    _ = compute_aerodynamic_coeffs_jax(ALPHA, BETA)

    # Benchmark runs
    print("Starting benchmark...")
    times = []
    for i in range(num_runs):
        start_time = time.time()
        _ = compute_aerodynamic_coeffs_jax(ALPHA, BETA)
        end_time = time.time()
        elapsed = (end_time - start_time) * 1000  # Convert to milliseconds
        times.append(elapsed)
        print(f"Run {i+1}: {elapsed:.2f} ms")

    # Calculate statistics
    avg_time = sum(times) / len(times)
    min_time = min(times)
    max_time = max(times)
    std_time = (sum((t - avg_time) ** 2 for t in times) / len(times)) ** 0.5

    print(f"\nJAX Timing Results:")
    print(f"Average: {avg_time:.2f} ms")
    print(f"Min: {min_time:.2f} ms")
    print(f"Max: {max_time:.2f} ms")
    print(f"Std Dev: {std_time:.2f} ms")
    print(f"Throughput: {mesh_size*mesh_size/avg_time:.0f} points/ms")

    return {
        'avg_time': avg_time,
        'min_time': min_time,
        'max_time': max_time,
        'std_time': std_time,
        'throughput': mesh_size*mesh_size/avg_time
    }


if __name__ == "__main__":
    # Run the JAX batched plotting function when script is executed directly
    benchmark_jax_timing()