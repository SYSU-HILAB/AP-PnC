"""
Body frame aerodynamic coefficient plotting functions.

Functions:
- plot_body_frame_coeffs(): Plot cx, cy, cz vs alpha (1x3 layout)
- plot_body_frame_derivatives(): Plot dcx/dalpha, dcy/dalpha, dcz/dalpha (1x3 layout)
- plot_body_frame_combined(): Combined 2x3 layout
"""

import os

import matplotlib.pyplot as plt
import numpy as np

BODY_FRAME_KEYS = (
    "alpha_deg",
    "beta_values",
    "cx",
    "cy",
    "cz",
    "dcx_dalpha",
    "dcy_dalpha",
    "dcz_dalpha",
)


def load_body_frame_data(
    data_file: str,
    auto_generate: bool = False,
    alpha_range: tuple[float, float] = (-30.0, 120.0),
    beta_values: np.ndarray | None = None,
) -> dict:
    """
    Load body frame data from .npy file. Auto-generates if file doesn't exist.

    Args:
        data_file: Path to .npy file containing body frame data
        auto_generate: If True, automatically generate data if file doesn't exist
        alpha_range: (min, max) alpha range for auto-generation (degrees)
        beta_values: Beta values for auto-generation (degrees)

    Returns:
        dict: Dictionary containing the body frame data

    Raises:
        FileNotFoundError: If data file doesn't exist and auto_generate=False
    """
    if not os.path.exists(data_file):
        if auto_generate:
            print(f"Data file not found: {data_file}")
            print("Auto-generating body frame data using CasADi backend...")

            from tooling.paper.methods.lu.gd_model_casadi import generate_body_frame_coeffs_data

            filename = os.path.splitext(os.path.basename(data_file))[0]
            return generate_body_frame_coeffs_data(
                alpha_range=alpha_range,
                beta_values=beta_values,
                filename=filename,
            )
        else:
            raise FileNotFoundError(f"Data file not found: {data_file}")

    return np.load(data_file, allow_pickle=True).item()


def plot_body_frame_coeffs(
    data_file: str | None = None,
    data: dict | None = None,
    show_plot: bool = True,
    auto_generate: bool = True,
    figsize: tuple[int, int] = (18, 6),
) -> dict:
    """
    Plot body frame aerodynamic coefficients (cx, cy, cz) across alpha.

    Creates one row with three subplots, each showing multiple lines for different beta values.

    Args:
        data_file: Path to .npy file containing body frame data. If None, data must be provided.
        data: Direct data dictionary. If provided, data_file is ignored.
        show_plot: Whether to display the plot. Default: True.
        auto_generate: Whether to auto-generate data if file not found or no data provided. Default: True.
        figsize: Figure size as (width, height). Default: (18, 6).

    Returns:
        dict: The body frame data used for plotting

    Raises:
        ValueError: If neither data_file nor data is provided and auto_generate=False
    """
    # Load data
    if data is None:
        if data_file is None:
            if auto_generate:
                # Generate default dataset
                from tooling.paper.methods.lu.gd_model_casadi import generate_body_frame_coeffs_data
                data = generate_body_frame_coeffs_data()
            else:
                raise ValueError("Either data_file or data must be provided")
        else:
            data = load_body_frame_data(data_file, auto_generate=auto_generate)

    # Validate data
    for key in BODY_FRAME_KEYS[:5]:  # Only need coeffs, not derivatives
        if key not in data:
            raise ValueError(f"Missing required key '{key}' in data")

    if data.get("data_type") != "body_frame_coefficients":
        raise ValueError(
            f"Expected data_type='body_frame_coefficients', got '{data.get('data_type')}'"
        )

    # Extract data
    alpha_deg = data["alpha_deg"]
    beta_values = data["beta_values"]
    cx = data["cx"]
    cy = data["cy"]
    cz = data["cz"]

    # Create figure with 1 row, 3 columns
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize)
    fig.suptitle(
        "Body Frame Aerodynamic Coefficients vs Angle of Attack",
        fontsize=16,
        fontweight="bold",
    )

    # Color map for different beta values
    colors = plt.cm.viridis(np.linspace(0, 1, len(beta_values)))

    # Plot cx
    for j, beta in enumerate(beta_values):
        label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
        ax1.plot(alpha_deg, cx[:, j], color=colors[j], linewidth=1.5, label=label)
    ax1.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax1.set_ylabel("cx", fontsize=12)
    ax1.set_title("Body Frame cx Coefficient", fontsize=14, fontweight="bold")
    ax1.grid(True, alpha=0.3, linestyle="--")
    ax1.axhline(y=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax1.axvline(x=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    if len(beta_values) <= 11:
        ax1.legend(loc="best", fontsize=9)

    # Plot cy
    for j, beta in enumerate(beta_values):
        label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
        ax2.plot(alpha_deg, cy[:, j], color=colors[j], linewidth=1.5, label=label)
    ax2.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax2.set_ylabel("cy", fontsize=12)
    ax2.set_title("Body Frame cy Coefficient", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3, linestyle="--")
    ax2.axhline(y=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax2.axvline(x=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    if len(beta_values) <= 11:
        ax2.legend(loc="best", fontsize=9)

    # Plot cz
    for j, beta in enumerate(beta_values):
        label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
        ax3.plot(alpha_deg, cz[:, j], color=colors[j], linewidth=1.5, label=label)
    ax3.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax3.set_ylabel("cz", fontsize=12)
    ax3.set_title("Body Frame cz Coefficient", fontsize=14, fontweight="bold")
    ax3.grid(True, alpha=0.3, linestyle="--")
    ax3.axhline(y=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax3.axvline(x=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    if len(beta_values) <= 11:
        ax3.legend(loc="best", fontsize=9)

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("Body frame coefficients plot completed!")
    return data


def plot_body_frame_derivatives(
    data_file: str | None = None,
    data: dict | None = None,
    show_plot: bool = True,
    auto_generate: bool = True,
    figsize: tuple[int, int] = (18, 6),
) -> dict:
    """
    Plot body frame aerodynamic coefficient derivatives (dcx/dalpha, dcy/dalpha, dcz/dalpha).

    Creates one row with three subplots, each showing multiple lines for different beta values.

    Args:
        data_file: Path to .npy file containing body frame data. If None, data must be provided.
        data: Direct data dictionary. If provided, data_file is ignored.
        show_plot: Whether to display the plot. Default: True.
        auto_generate: Whether to auto-generate data if file not found or no data provided. Default: True.
        figsize: Figure size as (width, height). Default: (18, 6).

    Returns:
        dict: The body frame data used for plotting

    Raises:
        ValueError: If neither data_file nor data is provided and auto_generate=False
    """
    # Load data
    if data is None:
        if data_file is None:
            if auto_generate:
                # Generate default dataset
                from tooling.paper.methods.lu.gd_model_casadi import generate_body_frame_coeffs_data
                data = generate_body_frame_coeffs_data()
            else:
                raise ValueError("Either data_file or data must be provided")
        else:
            data = load_body_frame_data(data_file, auto_generate=auto_generate)

    # Validate data
    for key in BODY_FRAME_KEYS:
        if key not in data:
            raise ValueError(f"Missing required key '{key}' in data")

    if data.get("data_type") != "body_frame_coefficients":
        raise ValueError(
            f"Expected data_type='body_frame_coefficients', got '{data.get('data_type')}'"
        )

    # Extract data
    alpha_deg = data["alpha_deg"]
    beta_values = data["beta_values"]
    dcx_dalpha = data["dcx_dalpha"]
    dcy_dalpha = data["dcy_dalpha"]
    dcz_dalpha = data["dcz_dalpha"]

    # Create figure with 1 row, 3 columns
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize)
    fig.suptitle(
        "Body Frame Aerodynamic Coefficient Derivatives vs Angle of Attack",
        fontsize=16,
        fontweight="bold",
    )

    # Color map for different beta values
    colors = plt.cm.plasma(np.linspace(0, 1, len(beta_values)))

    # Plot dcx/dalpha
    for j, beta in enumerate(beta_values):
        label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
        ax1.plot(alpha_deg, dcx_dalpha[:, j], color=colors[j], linewidth=1.5, label=label)
    ax1.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax1.set_ylabel("\u2202cx/\u2202\u03b1 [1/rad]", fontsize=12)
    ax1.set_title("\u2202cx/\u2202\u03b1 Derivative", fontsize=14, fontweight="bold")
    ax1.grid(True, alpha=0.3, linestyle="--")
    ax1.axhline(y=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax1.axvline(x=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    if len(beta_values) <= 11:
        ax1.legend(loc="best", fontsize=9)

    # Plot dcy/dalpha
    for j, beta in enumerate(beta_values):
        label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
        ax2.plot(alpha_deg, dcy_dalpha[:, j], color=colors[j], linewidth=1.5, label=label)
    ax2.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax2.set_ylabel("\u2202cy/\u2202\u03b1 [1/rad]", fontsize=12)
    ax2.set_title("\u2202cy/\u2202\u03b1 Derivative", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3, linestyle="--")
    ax2.axhline(y=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax2.axvline(x=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    if len(beta_values) <= 11:
        ax2.legend(loc="best", fontsize=9)

    # Plot dcz/dalpha
    for j, beta in enumerate(beta_values):
        label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
        ax3.plot(alpha_deg, dcz_dalpha[:, j], color=colors[j], linewidth=1.5, label=label)
    ax3.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax3.set_ylabel("\u2202cz/\u2202\u03b1 [1/rad]", fontsize=12)
    ax3.set_title("\u2202cz/\u2202\u03b1 Derivative", fontsize=14, fontweight="bold")
    ax3.grid(True, alpha=0.3, linestyle="--")
    ax3.axhline(y=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax3.axvline(x=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    if len(beta_values) <= 11:
        ax3.legend(loc="best", fontsize=9)

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("Body frame derivatives plot completed!")
    return data


def plot_body_frame_combined(
    data_file: str | None = None,
    data: dict | None = None,
    show_plot: bool = True,
    auto_generate: bool = True,
    figsize: tuple[int, int] = (18, 12),
) -> dict:
    """
    Plot both body frame coefficients and derivatives in a 2x3 grid layout.

    Top row: cx, cy, cz coefficients
    Bottom row: dcx/dalpha, dcy/dalpha, dcz/dalpha derivatives

    Args:
        data_file: Path to .npy file containing body frame data. If None, data must be provided.
        data: Direct data dictionary. If provided, data_file is ignored.
        show_plot: Whether to display the plot. Default: True.
        auto_generate: Whether to auto-generate data if file not found or no data provided. Default: True.
        figsize: Figure size as (width, height). Default: (18, 12).

    Returns:
        dict: The body frame data used for plotting

    Raises:
        ValueError: If neither data_file nor data is provided and auto_generate=False
    """
    # Load data
    if data is None:
        if data_file is None:
            if auto_generate:
                from tooling.paper.methods.lu.gd_model_casadi import generate_body_frame_coeffs_data
                data = generate_body_frame_coeffs_data()
            else:
                raise ValueError("Either data_file or data must be provided")
        else:
            data = load_body_frame_data(data_file, auto_generate=auto_generate)

    for key in BODY_FRAME_KEYS:
        if key not in data:
            raise ValueError(f"Missing required key '{key}' in data")

    alpha_deg = data["alpha_deg"]
    beta_values = data["beta_values"]
    cx, cy, cz = data["cx"], data["cy"], data["cz"]
    dcx_dalpha, dcy_dalpha, dcz_dalpha = (
        data["dcx_dalpha"],
        data["dcy_dalpha"],
        data["dcz_dalpha"],
    )

    fig, axes = plt.subplots(2, 3, figsize=figsize)
    fig.suptitle(
        "Body Frame Aerodynamic Coefficients and Derivatives",
        fontsize=16,
        fontweight="bold",
    )

    coeff_colors = plt.cm.viridis(np.linspace(0, 1, len(beta_values)))
    deriv_colors = plt.cm.plasma(np.linspace(0, 1, len(beta_values)))

    for col, (coeff, name) in enumerate([(cx, "cx"), (cy, "cy"), (cz, "cz")]):
        ax = axes[0, col]
        for j, beta in enumerate(beta_values):
            label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
            ax.plot(alpha_deg, coeff[:, j], color=coeff_colors[j], linewidth=1.5, label=label)
        ax.set_xlabel("\u03b1 [degrees]", fontsize=11)
        ax.set_ylabel(name, fontsize=11)
        ax.set_title(f"{name} Coefficient", fontsize=12, fontweight="bold")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
        ax.axvline(x=0, color="k", linestyle="-", alpha=0.3)
        if len(beta_values) <= 11:
            ax.legend(loc="best", fontsize=8)

    for col, (deriv, name, ylabel) in enumerate([
        (dcx_dalpha, "\u2202cx/\u2202\u03b1", "\u2202cx/\u2202\u03b1 [1/rad]"),
        (dcy_dalpha, "\u2202cy/\u2202\u03b1", "\u2202cy/\u2202\u03b1 [1/rad]"),
        (dcz_dalpha, "\u2202cz/\u2202\u03b1", "\u2202cz/\u2202\u03b1 [1/rad]"),
    ]):
        ax = axes[1, col]
        for j, beta in enumerate(beta_values):
            label = f"\u03b2 = {beta:.1f}\u00b0" if len(beta_values) <= 11 else None
            ax.plot(alpha_deg, deriv[:, j], color=deriv_colors[j], linewidth=1.5, label=label)
        ax.set_xlabel("\u03b1 [degrees]", fontsize=11)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(f"{name} Derivative", fontsize=12, fontweight="bold")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
        ax.axvline(x=0, color="k", linestyle="-", alpha=0.3)
        if len(beta_values) <= 11:
            ax.legend(loc="best", fontsize=8)

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("Combined body frame plot completed!")
    return data


def plot_body_frame_twins_beta_zero(
    data_file: str | None = None,
    data: dict | None = None,
    show_plot: bool = True,
    auto_generate: bool = True,
    figsize: tuple[int, int] = (18, 6),
) -> dict:
    """
    Plot body frame coefficients and their derivatives on twin axes at beta=0.

    Creates 3 subplots (1 row, 3 columns), each with twin y-axes:
    - Left axis: coefficient (cx, cy, or cz)
    - Right axis: derivative (dcoeff/dalpha)

    Args:
        data_file: Path to .npy file containing body frame data. If None, data must be provided.
        data: Direct data dictionary. If provided, data_file is ignored.
        show_plot: Whether to display the plot. Default: True.
        auto_generate: Whether to auto-generate data if file not found or no data provided. Default: True.
        figsize: Figure size as (width, height). Default: (18, 6).

    Returns:
        dict: The body frame data used for plotting

    Raises:
        ValueError: If neither data_file nor data is provided and auto_generate=False
    """
    if data is None:
        if data_file is None:
            if auto_generate:
                from tooling.paper.methods.lu.gd_model_casadi import generate_body_frame_coeffs_data
                data = generate_body_frame_coeffs_data()
            else:
                raise ValueError("Either data_file or data must be provided")
        else:
            data = load_body_frame_data(data_file, auto_generate=auto_generate)

    for key in BODY_FRAME_KEYS:
        if key not in data:
            raise ValueError(f"Missing required key '{key}' in data")

    alpha_deg = data["alpha_deg"]
    beta_values = data["beta_values"]
    cx, cy, cz = data["cx"], data["cy"], data["cz"]
    dcx_dalpha, dcy_dalpha, dcz_dalpha = (
        data["dcx_dalpha"],
        data["dcy_dalpha"],
        data["dcz_dalpha"],
    )

    beta_zero_idx = np.argmin(np.abs(beta_values))
    beta_zero_val = beta_values[beta_zero_idx]

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize)
    fig.suptitle(
        f"Body Frame Coefficients & Derivatives at \u03b2 \u2248 {beta_zero_val:.1f}\u00b0",
        fontsize=16,
        fontweight="bold",
    )

    cx_beta0 = cx[:, beta_zero_idx]
    cy_beta0 = cy[:, beta_zero_idx]
    cz_beta0 = cz[:, beta_zero_idx]
    dcx_beta0 = dcx_dalpha[:, beta_zero_idx]
    dcy_beta0 = dcy_dalpha[:, beta_zero_idx]
    dcz_beta0 = dcz_dalpha[:, beta_zero_idx]

    color1 = "tab:blue"
    ax1.plot(alpha_deg, cx_beta0, color=color1, linewidth=2, label="cx")
    ax1.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax1.set_ylabel("cx", fontsize=12, color=color1)
    ax1.tick_params(axis="y", labelcolor=color1)
    ax1.grid(True, alpha=0.3, linestyle="--")
    ax1.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    ax1.axvline(x=0, color="k", linestyle="-", alpha=0.3)
    ax1.legend(loc="upper left", fontsize=10)

    ax1_twin = ax1.twinx()
    color1_twin = "tab:orange"
    ax1_twin.plot(alpha_deg, dcx_beta0, color=color1_twin, linewidth=2, linestyle="--", label="\u2202cx/\u2202\u03b1")
    ax1_twin.set_ylabel("\u2202cx/\u2202\u03b1 [1/rad]", fontsize=12, color=color1_twin)
    ax1_twin.tick_params(axis="y", labelcolor=color1_twin)
    ax1_twin.axhline(y=0, color=color1_twin, linestyle="-", alpha=0.3, linewidth=1)
    ax1_twin.legend(loc="upper right", fontsize=10)
    ax1.set_title("cx & \u2202cx/\u2202\u03b1", fontsize=14, fontweight="bold")

    color2 = "tab:green"
    ax2.plot(alpha_deg, cy_beta0, color=color2, linewidth=2, label="cy")
    ax2.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax2.set_ylabel("cy", fontsize=12, color=color2)
    ax2.tick_params(axis="y", labelcolor=color2)
    ax2.grid(True, alpha=0.3, linestyle="--")
    ax2.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    ax2.axvline(x=0, color="k", linestyle="-", alpha=0.3)
    ax2.legend(loc="upper left", fontsize=10)

    ax2_twin = ax2.twinx()
    color2_twin = "tab:red"
    ax2_twin.plot(alpha_deg, dcy_beta0, color=color2_twin, linewidth=2, linestyle="--", label="\u2202cy/\u2202\u03b1")
    ax2_twin.set_ylabel("\u2202cy/\u2202\u03b1 [1/rad]", fontsize=12, color=color2_twin)
    ax2_twin.tick_params(axis="y", labelcolor=color2_twin)
    ax2_twin.axhline(y=0, color=color2_twin, linestyle="-", alpha=0.3, linewidth=1)
    ax2_twin.legend(loc="upper right", fontsize=10)
    ax2.set_title("cy & \u2202cy/\u2202\u03b1", fontsize=14, fontweight="bold")

    color3 = "tab:purple"
    ax3.plot(alpha_deg, cz_beta0, color=color3, linewidth=2, label="cz")
    ax3.set_xlabel("\u03b1 [degrees]", fontsize=12)
    ax3.set_ylabel("cz", fontsize=12, color=color3)
    ax3.tick_params(axis="y", labelcolor=color3)
    ax3.grid(True, alpha=0.3, linestyle="--")
    ax3.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    ax3.axvline(x=0, color="k", linestyle="-", alpha=0.3)
    ax3.legend(loc="upper left", fontsize=10)

    ax3_twin = ax3.twinx()
    color3_twin = "tab:brown"
    ax3_twin.plot(alpha_deg, dcz_beta0, color=color3_twin, linewidth=2, linestyle="--", label="\u2202cz/\u2202\u03b1")
    ax3_twin.set_ylabel("\u2202cz/\u2202\u03b1 [1/rad]", fontsize=12, color=color3_twin)
    ax3_twin.tick_params(axis="y", labelcolor=color3_twin)
    ax3_twin.axhline(y=0, color=color3_twin, linestyle="-", alpha=0.3, linewidth=1)
    ax3_twin.legend(loc="upper right", fontsize=10)
    ax3.set_title("cz & \u2202cz/\u2202\u03b1", fontsize=14, fontweight="bold")

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("Twin-axes plot completed!")
    return data


def plot_cy_vs_beta_jax(
    alpha_degrees: list[float] | None = None,
    beta_range: tuple[float, float] = (-90.0, 90.0),
    num_beta_points: int = 1801,
    show_plot: bool = True,
    figsize: tuple[int, int] = (12, 8),
) -> dict:
    """
    Plot cy (side force coefficient) vs beta for different alpha values using JAX.

    This is the inverse of the typical cy vs alpha plot - here beta is on the x-axis
    and we show multiple curves for different alpha values.

    Args:
        alpha_degrees: List of alpha values in degrees to plot. Default: [0, 15, 30, 45, 60, 75, 90, 105, 120]
        beta_range: (min, max) beta range in degrees. Default: (-90, 90)
        num_beta_points: Number of beta points to compute. Default: 1801 (0.1 degree resolution)
        show_plot: Whether to display the plot. Default: True.
        figsize: Figure size as (width, height). Default: (12, 8).

    Returns:
        dict: Dictionary containing beta_deg and cy_values (2D array with shape (num_beta, num_alpha))
    """
    import jax.numpy as jnp

    from tooling.paper.methods.lu.gd_model_jax import deg2rad_batch, kCY, reg_CY_batch

    if alpha_degrees is None:
        alpha_degrees = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0]

    print(f"Computing cy vs beta for alpha values: {alpha_degrees}")
    print(f"Beta range: {beta_range} degrees, {num_beta_points} points")

    beta_deg = jnp.linspace(beta_range[0], beta_range[1], num_beta_points)
    beta_rad = deg2rad_batch(beta_deg)

    alpha_deg_arr = jnp.array(alpha_degrees)
    alpha_rad_arr = deg2rad_batch(alpha_deg_arr)

    ALPHA_RAD, BETA_RAD = jnp.meshgrid(alpha_rad_arr, beta_rad, indexing="ij")

    print("Computing CY coefficients using JAX...")
    reg_cy = reg_CY_batch(ALPHA_RAD, BETA_RAD)
    cy_values = jnp.sum(reg_cy * kCY, axis=-1)

    beta_deg_np = np.array(beta_deg)
    cy_values_np = np.array(cy_values)

    fig, ax = plt.subplots(figsize=figsize)
    fig.suptitle(
        "Side Force Coefficient cy vs Sideslip Angle \u03b2",
        fontsize=16,
        fontweight="bold",
    )

    colors = plt.cm.plasma(np.linspace(0, 1, len(alpha_degrees)))

    for i, (alpha_val, color) in enumerate(zip(alpha_degrees, colors, strict=False)):
        label = f"\u03b1 = {alpha_val:.0f}\u00b0"
        ax.plot(
            beta_deg_np,
            cy_values_np[i, :],
            color=color,
            linewidth=2,
            label=label,
        )

    ax.set_xlabel("Sideslip Angle \u03b2 [degrees]", fontsize=14)
    ax.set_ylabel("Side Force Coefficient cy", fontsize=14)
    ax.set_title("cy vs \u03b2 for Different \u03b1 Values", fontsize=15, fontweight="bold")
    ax.grid(True, alpha=0.3, linestyle="--")
    ax.axhline(y=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax.axvline(x=0, color="k", linestyle="-", alpha=0.3, linewidth=1)
    ax.legend(loc="best", fontsize=11, framealpha=0.9)

    plt.tight_layout()

    if show_plot:
        plt.show()

    print("cy vs beta plot completed!")
    return {
        "beta_deg": beta_deg_np,
        "cy_values": cy_values_np,
        "alpha_degrees": np.array(alpha_degrees),
    }
