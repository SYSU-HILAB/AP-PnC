# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# Authors:
# Erchao Rong: rongerch@outlook.com
# Zihao Liu: liuzh297@gmail.com
# Junning Liang: gordonliang27@foxmail.com
#
# PyTorch batched implementation for efficient aerodynamic coefficient calculations
#
import math

import matplotlib.pyplot as plt
import torch
import torch.jit

from .aero_coefficients import get_torch_coefficients

COEFFICIENTS = get_torch_coefficients()
kCL = COEFFICIENTS.kCL
kCD = COEFFICIENTS.kCD
kCY = COEFFICIENTS.kCY
kCLL = COEFFICIENTS.kCLL
kCm = COEFFICIENTS.kCm
kCn = COEFFICIENTS.kCn


@torch.jit.script
def logistic_func_batch(
    alpha: torch.Tensor, alpha1: float, k1: float, alpha2: float, k2: float
) -> torch.Tensor:
    """Batched logistic function with JIT compilation"""
    P1 = 1.0 / (1 + torch.exp(k1 * (alpha - alpha1)))
    P2 = 1.0 / (1 + torch.exp(-k2 * (alpha - alpha2)))
    yval = 1 - P1 - P2
    return yval


def deg2rad_batch(deg):
    """Batched degree to radian conversion"""
    if isinstance(deg, int | float):
        # Handle scalar input
        return torch.tensor(deg * (math.pi / 180.0))
    else:
        # Handle tensor input
        return deg * (math.pi / 180.0)


def reg_CL_batch(alpha, beta):
    """Batched regression calculation for CL coefficient"""
    sa = torch.sin(alpha)
    ca = torch.cos(alpha)
    sb = torch.sin(beta)
    cb = torch.cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2
    sa3 = sa**3
    ca3 = ca**3

    comp_B = torch.stack([cb2, alpha * cb2], dim=-1)

    comp_N = torch.stack(
        [
            cb2 * ca3,
            ca * sb2,
            ca * cb2 * sa2,
            sa * ca2 * cb2,
            sa * sb2,
            sa3 * cb2,
            torch.abs(sb) * cb * ca2,
            sa * ca * cb * torch.abs(sb),
            torch.abs(sb) * cb * sa2,
        ],
        dim=-1,
    )

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], dim=-1)
    reg_B1 = w_B1.unsqueeze(-1) * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-20), 0.5)
    reg_N11 = w_N11.unsqueeze(-1) * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2.unsqueeze(-1) * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(170), 1)
    reg_N2 = w_N2.unsqueeze(-1) * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], dim=-1)
    reg_B3 = w_B3.unsqueeze(-1) * comp_B3

    reg = torch.cat([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3], dim=-1)
    return reg


def reg_CD_batch(alpha, beta):
    """Batched regression calculation for CD coefficient"""
    sa = torch.sin(alpha)
    ca = torch.cos(alpha)
    sb = torch.sin(beta)
    cb = torch.cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2
    sa3 = sa**3
    ca3 = ca**3

    comp_B = torch.stack([cb2, alpha * cb2], dim=-1)

    comp_N = torch.stack(
        [
            cb2 * ca3,
            ca * sb2,
            ca * cb2 * sa2,
            sa * ca2 * cb2,
            sa * sb2,
            sa3 * cb2,
            torch.abs(sb) * cb * ca2,
            sa * ca * cb * torch.abs(sb),
            torch.abs(sb) * cb * sa2,
        ],
        dim=-1,
    )

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], dim=-1)
    reg_B1 = w_B1.unsqueeze(-1) * comp_B1

    w_N1 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-10), 0.5)
    reg_N1 = w_N1.unsqueeze(-1) * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2.unsqueeze(-1) * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(170), 1)
    reg_N2 = w_N2.unsqueeze(-1) * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], dim=-1)
    reg_B3 = w_B3.unsqueeze(-1) * comp_B3

    reg = torch.cat([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3], dim=-1)
    return reg


def reg_CY_batch(alpha, beta):
    """Batched regression calculation for CY coefficient"""
    sa = torch.sin(alpha)
    ca = torch.cos(alpha)
    sb = torch.sin(beta)
    cb = torch.cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    signb = beta * torch.abs(beta)

    comp_N = torch.stack([signb * sb2, sb * cb * ca, sb * cb * sa], dim=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], dim=-1)
    reg_B1 = w_B1.unsqueeze(-1) * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-90), 1)
    reg_N11 = w_N11.unsqueeze(-1) * comp_N

    w_N12 = logistic_func_batch(alpha, deg2rad_batch(-90), 1, deg2rad_batch(-11), 1)
    reg_N12 = w_N12.unsqueeze(-1) * comp_N

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(20), 1, deg2rad_batch(170), 1)
    reg_N2 = w_N2.unsqueeze(-1) * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], dim=-1)
    reg_B3 = w_B3.unsqueeze(-1) * comp_B3

    reg = torch.cat([reg_B1, reg_N11, reg_N12, reg_N2, reg_B3], dim=-1)
    return reg


def reg_Cll_batch(alpha, beta):
    """Batched regression calculation for Cl coefficient"""
    sa = torch.sin(alpha)
    ca = torch.cos(alpha)
    sb = torch.sin(beta)
    cb = torch.cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    signb = beta * torch.abs(beta)

    comp_B = torch.stack([cb2, alpha * cb2], dim=-1)
    comp_N = torch.stack([signb * sb2, sb * cb * ca, sb * cb * sa], dim=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], dim=-1)
    reg_B1 = w_B1.unsqueeze(-1) * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-90), 5)
    reg_N11 = w_N11.unsqueeze(-1) * comp_N

    w_N12 = logistic_func_batch(alpha, deg2rad_batch(-90), 5, deg2rad_batch(-20), 0.5)
    reg_N12 = w_N12.unsqueeze(-1) * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2.unsqueeze(-1) * comp_B

    w_N22 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(90), 5)
    reg_N22 = w_N22.unsqueeze(-1) * comp_N

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(90), 5, deg2rad_batch(170), 1)
    reg_N2 = w_N2.unsqueeze(-1) * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], dim=-1)
    reg_B3 = w_B3.unsqueeze(-1) * comp_B3

    reg = torch.cat([reg_B1, reg_N11, reg_N12, reg_B2, reg_N2, reg_N22, reg_B3], dim=-1)
    return reg


def reg_Cm_batch(alpha, beta):
    """Batched regression calculation for Cm coefficient"""
    sa = torch.sin(alpha)
    ca = torch.cos(alpha)
    sb = torch.sin(beta)
    cb = torch.cos(beta)
    sa2 = sa**2
    ca2 = ca**2
    sb2 = sb**2
    cb2 = cb**2

    comp_B = torch.stack([cb2, alpha * cb2], dim=-1)
    comp_N = torch.stack(
        [
            cb2 * ca2,
            sb2,
            cb2 * sa2,
            torch.abs(sb) * cb * ca,
            sa * ca * cb2,
            torch.abs(sb) * cb * sa,
        ],
        dim=-1,
    )

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], dim=-1)
    reg_B1 = w_B1.unsqueeze(-1) * comp_B1

    w_N11 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-20), 0.5)
    reg_N11 = w_N11.unsqueeze(-1) * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-11), 20, deg2rad_batch(18), 10)
    reg_B2 = w_B2.unsqueeze(-1) * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(18), 10, deg2rad_batch(170), 1)
    reg_N2 = w_N2.unsqueeze(-1) * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], dim=-1)
    reg_B3 = w_B3.unsqueeze(-1) * comp_B3

    reg = torch.cat([reg_B1, reg_N11, reg_B2, reg_N2, reg_B3], dim=-1)
    return reg


def reg_Cn_batch(alpha, beta):
    """Batched regression calculation for Cn coefficient"""
    sa = torch.sin(alpha)
    ca = torch.cos(alpha)
    sb = torch.sin(beta)
    cb = torch.cos(beta)
    sb2 = sb**2
    cb2 = cb**2
    signb = beta * torch.abs(beta)

    comp_B = torch.stack([cb2, alpha * cb2], dim=-1)
    comp_N = torch.stack([signb * sb2, sb * cb * ca, sb * cb * sa], dim=-1)

    w_B1 = logistic_func_batch(alpha, deg2rad_batch(-190), 1, deg2rad_batch(-170), 1)
    comp_B1 = torch.stack([cb2, (alpha + deg2rad_batch(180)) * cb2], dim=-1)
    reg_B1 = w_B1.unsqueeze(-1) * comp_B1

    w_N1 = logistic_func_batch(alpha, deg2rad_batch(-170), 1, deg2rad_batch(-10), 10)
    reg_N1 = w_N1.unsqueeze(-1) * comp_N

    w_B2 = logistic_func_batch(alpha, deg2rad_batch(-10), 10, deg2rad_batch(18), 10)
    reg_B2 = w_B2.unsqueeze(-1) * comp_B

    w_N2 = logistic_func_batch(alpha, deg2rad_batch(0), 0.5, deg2rad_batch(170), 1)
    reg_N2 = w_N2.unsqueeze(-1) * comp_N

    w_B3 = logistic_func_batch(alpha, deg2rad_batch(170), 1, deg2rad_batch(190), 1)
    comp_B3 = torch.stack([cb2, (alpha - deg2rad_batch(180)) * cb2], dim=-1)
    reg_B3 = w_B3.unsqueeze(-1) * comp_B3

    reg = torch.cat([reg_B1, reg_N1, reg_B2, reg_N2, reg_B3], dim=-1)
    return reg


def compute_aerodynamic_coeffs_torch(
    ALPHA: torch.Tensor,
    BETA: torch.Tensor,
    kCL: torch.Tensor,
    kCD: torch.Tensor,
    kCY: torch.Tensor,
    kCLL: torch.Tensor,
    kCm: torch.Tensor,
    kCn: torch.Tensor,
):
    """
    Compute all aerodynamic coefficients using PyTorch.
    Returns all coefficients without plotting.
    Note: JIT disabled due to compatibility issues with mixed tensor/scalar operations.
    """
    # Calculate coefficients in batches
    reg_cl = reg_CL_batch(ALPHA, BETA)
    reg_cd = reg_CD_batch(ALPHA, BETA)
    reg_cy = reg_CY_batch(ALPHA, BETA)
    reg_cll = reg_Cll_batch(ALPHA, BETA)
    reg_cm = reg_Cm_batch(ALPHA, BETA)
    reg_cn = reg_Cn_batch(ALPHA, BETA)

    # Batched coefficient calculation using matrix multiplication
    CL = torch.sum(reg_cl * kCL.view(1, 1, -1), dim=-1)
    CD = torch.sum(reg_cd * kCD.view(1, 1, -1), dim=-1)
    CY = torch.sum(reg_cy * kCY.view(1, 1, -1), dim=-1)
    CLL = torch.sum(reg_cll * kCLL.view(1, 1, -1), dim=-1)
    CM = torch.sum(reg_cm * kCm.view(1, 1, -1), dim=-1)
    CN = torch.sum(reg_cn * kCn.view(1, 1, -1), dim=-1)

    return CL, CD, CY, CLL, CM, CN


def plot_aerodynamic_mesh_torch(azimuth=-127.5):
    """
    Generate 3D mesh plots of aerodynamic coefficients using batched PyTorch calculations.
    Much faster than the original numpy version.

    Args:
        azimuth (float): Azimuth angle for 3D view in degrees (default: -127.5)
    """
    print("Setting up mesh grid...")

    # Force GPU usage
    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA is not available. Please install PyTorch with CUDA support."
        )

    device = torch.device("cuda")
    print(f"Using device: {device}")
    print(f"CUDA device: {torch.cuda.get_device_name()}")
    print(f"CUDA version: {torch.version.cuda}")

    # Generate ranges for alpha and beta on GPU directly
    alpha_deg = torch.linspace(
        -180, 180, 361, device=device
    )  # 1-degree increments on GPU
    beta_deg = torch.linspace(-90, 90, 361, device=device)  # 1-degree increments on GPU

    alpha_rad = deg2rad_batch(alpha_deg)
    beta_rad = deg2rad_batch(beta_deg)

    # Create meshgrid on GPU
    ALPHA_DEG, BETA_DEG = torch.meshgrid(alpha_deg, beta_deg, indexing="ij")
    ALPHA, BETA = torch.meshgrid(alpha_rad, beta_rad, indexing="ij")

    # Move coefficient tensors to GPU
    kCL_device = kCL.to(device)
    kCD_device = kCD.to(device)
    kCY_device = kCY.to(device)
    kCLL_device = kCLL.to(device)
    kCm_device = kCm.to(device)
    kCn_device = kCn.to(device)

    print("Computing aerodynamic coefficients in batches...")

    # Calculate coefficients in batches
    reg_cl = reg_CL_batch(ALPHA, BETA)
    reg_cd = reg_CD_batch(ALPHA, BETA)
    reg_cy = reg_CY_batch(ALPHA, BETA)
    reg_cll = reg_Cll_batch(ALPHA, BETA)
    reg_cm = reg_Cm_batch(ALPHA, BETA)
    reg_cn = reg_Cn_batch(ALPHA, BETA)

    # Batched coefficient calculation using matrix multiplication
    CL = torch.sum(reg_cl * kCL_device.unsqueeze(0).unsqueeze(0), dim=-1)
    CD = torch.sum(reg_cd * kCD_device.unsqueeze(0).unsqueeze(0), dim=-1)
    CY = torch.sum(reg_cy * kCY_device.unsqueeze(0).unsqueeze(0), dim=-1)
    CLL = torch.sum(reg_cll * kCLL_device.unsqueeze(0).unsqueeze(0), dim=-1)
    CM = torch.sum(reg_cm * kCm_device.unsqueeze(0).unsqueeze(0), dim=-1)
    CN = torch.sum(reg_cn * kCn_device.unsqueeze(0).unsqueeze(0), dim=-1)

    # Move back to CPU and convert to numpy for plotting
    print("Moving results to CPU for plotting...")
    ALPHA_DEG_np = ALPHA_DEG.cpu().numpy()
    BETA_DEG_np = BETA_DEG.cpu().numpy()
    CL_np = CL.cpu().numpy()
    CD_np = CD.cpu().numpy()
    CY_np = CY.cpu().numpy()
    CLL_np = CLL.cpu().numpy()
    CM_np = CM.cpu().numpy()
    CN_np = CN.cpu().numpy()

    print("Creating plots...")
    # Create figure with 2x3 subplots
    fig = plt.figure(figsize=(18, 8))
    fig.suptitle(
        "Aerodynamic Force and Moment Coefficient Wind Tunnel Curves", fontsize=16
    )

    # Row 1: C_L, C_m, C_D
    # C_L subplot (position 1)
    ax1 = fig.add_subplot(2, 3, 1, projection="3d")
    ax1.view_init(elev=30, azim=azimuth)
    surf1 = ax1.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CL_np, cmap="viridis", alpha=0.8
    )
    ax1.set_xlabel("alpha[deg]")
    ax1.set_ylabel("beta[deg]")
    ax1.set_zlabel("C_L")
    ax1.set_title("Lift Coefficient C_L")
    plt.colorbar(surf1, ax=ax1, shrink=0.5)

    # C_m subplot (position 2)
    ax2 = fig.add_subplot(2, 3, 2, projection="3d")
    ax2.view_init(elev=30, azim=azimuth)
    surf2 = ax2.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CM_np, cmap="viridis", alpha=0.8
    )
    ax2.set_xlabel("alpha[deg]")
    ax2.set_ylabel("beta[deg]")
    ax2.set_zlabel("C_m")
    ax2.set_title("Pitching Moment Coefficient C_m")
    plt.colorbar(surf2, ax=ax2, shrink=0.5)

    # C_D subplot (position 3)
    ax3 = fig.add_subplot(2, 3, 3, projection="3d")
    ax3.view_init(elev=30, azim=azimuth)
    surf3 = ax3.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CD_np, cmap="viridis", alpha=0.8
    )
    ax3.set_xlabel("alpha[deg]")
    ax3.set_ylabel("beta[deg]")
    ax3.set_zlabel("C_D")
    ax3.set_title("Drag Coefficient C_D")
    plt.colorbar(surf3, ax=ax3, shrink=0.5)

    # Row 2: C_ll, C_Y, C_n
    # C_ll subplot (position 4)
    ax4 = fig.add_subplot(2, 3, 4, projection="3d")
    ax4.view_init(elev=30, azim=azimuth)
    surf4 = ax4.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CLL_np, cmap="viridis", alpha=0.8
    )
    ax4.set_xlabel("alpha[deg]")
    ax4.set_ylabel("beta[deg]")
    ax4.set_zlabel("C_l")
    ax4.set_title("Rolling Moment Coefficient C_l")
    plt.colorbar(surf4, ax=ax4, shrink=0.5)

    # C_Y subplot (position 5)
    ax5 = fig.add_subplot(2, 3, 5, projection="3d")
    ax5.view_init(elev=30, azim=azimuth)
    surf5 = ax5.plot_surface(
        ALPHA_DEG_np, BETA_DEG_np, CY_np, cmap="viridis", alpha=0.8
    )
    ax5.set_xlabel("alpha[deg]")
    ax5.set_ylabel("beta[deg]")
    ax5.set_zlabel("C_Y")
    ax5.set_title("Side Force Coefficient C_Y")
    plt.colorbar(surf5, ax=ax5, shrink=0.5)

    # C_n subplot (position 6)
    ax6 = fig.add_subplot(2, 3, 6, projection="3d")
    ax6.view_init(elev=30, azim=azimuth)
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

    print("PyTorch batched mesh plotting completed successfully!")

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


def generate_mesh_data_torch(
    filename="mesh_aero_data_torch", cache_dir=None, mesh_size=361
):
    """
    Generate 3D mesh data for aerodynamic coefficients using PyTorch.
    Saves data to .npy file for later plotting.

    Args:
        filename (str): Name for the data file (without extension)
        cache_dir (str, optional): Custom cache directory
        mesh_size (int): Size of the mesh grid (mesh_size x mesh_size points)

    Returns:
        str: Path to the saved data file
    """
    print("Generating PyTorch 3D mesh data...")

    # Import data utilities
    from ...plotting.data_utils import save_mesh_data

    # Determine device
    if torch.cuda.is_available():
        device = torch.device("cuda")
        print(f"Using CUDA device: {torch.cuda.get_device_name()}")
    else:
        device = torch.device("cpu")
        print("CUDA not available, using CPU")

    # Generate ranges for alpha and beta on device
    alpha_deg = torch.linspace(-180, 180, mesh_size, device=device)
    beta_deg = torch.linspace(-90, 90, mesh_size, device=device)

    alpha_rad = deg2rad_batch(alpha_deg)
    beta_rad = deg2rad_batch(beta_deg)

    # Create meshgrid on device
    ALPHA_DEG, BETA_DEG = torch.meshgrid(alpha_deg, beta_deg, indexing="ij")
    ALPHA, BETA = torch.meshgrid(alpha_rad, beta_rad, indexing="ij")

    # Move coefficient tensors to device
    kCL_device = kCL.to(device)
    kCD_device = kCD.to(device)
    kCY_device = kCY.to(device)
    kCLL_device = kCLL.to(device)
    kCm_device = kCm.to(device)
    kCn_device = kCn.to(device)

    print("Computing aerodynamic coefficients in batches...")

    # Calculate coefficients in batches
    CL, CD, CY, CLL, CM, CN = compute_aerodynamic_coeffs_torch(
        ALPHA,
        BETA,
        kCL_device,
        kCD_device,
        kCY_device,
        kCLL_device,
        kCm_device,
        kCn_device,
    )

    # Move back to CPU and convert to numpy for saving
    ALPHA_DEG_np = ALPHA_DEG.cpu().numpy()
    BETA_DEG_np = BETA_DEG.cpu().numpy()
    CL_np = CL.cpu().numpy()
    CD_np = CD.cpu().numpy()
    CY_np = CY.cpu().numpy()
    CLL_np = CLL.cpu().numpy()
    CM_np = CM.cpu().numpy()
    CN_np = CN.cpu().numpy()

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
        filename,
        cache_dir,
        backend="pytorch",
    )

    print("PyTorch mesh data generation completed!")
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


if __name__ == "__main__":
    # Run the PyTorch batched plotting function when script is executed directly
    plot_aerodynamic_mesh_torch()
