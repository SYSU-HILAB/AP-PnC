"""
Neural Network-Based Trim Analysis Tool

Computes steady-state flight conditions using MLP neural network aerodynamic models.
Solves the force balance equation for level flight across a velocity range using
brentq root-finding method.

Trim Equation (Force Balance Only):
    cz(α) * V² / mass + g * cos(α) = 0

Output:
    - CSV files: velocity_ms, alpha_deg, cx, cz, fx_per_mass, fz_per_mass, dfx_dalpha, dfz_dalpha, residual
    - 2x2 plots: cx vs AoA, cz vs AoA, ∂fx/mass vs AoA, ∂fz/mass vs AoA

Usage:
    # Run with default parameters
    uv run python test/aerodynamics/test_nn_trim_analysis.py

    # Run with custom parameters
    uv run python test/aerodynamics/test_nn_trim_analysis.py --mass 2.5 --v-max 15.0 --num-points 150

    # Generate plots only
    uv run python test/aerodynamics/test_nn_trim_analysis.py --plot-only
"""

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from aerodynamics.inference.derivatives import load_aero_derivatives
from aerodynamics.inference.predictor import load_aerodynamics_mlp
from scipy.optimize import root_scalar

from tooling.env import artifact_path


@dataclass
class TrimResults:
    """Container for trim analysis results.

    Attributes:
        velocities: Velocity array in m/s
        alphas: Angle of attack in radians (NaN for velocities with no solution)
        cx: Drag coefficient (NaN for velocities with no solution)
        cz: Lift coefficient (NaN for velocities with no solution)
        fx: X-force per mass (cx * V² / mass, NaN for velocities with no solution)
        fz: Z-force per mass (cz * V² / mass, NaN for velocities with no solution)
        dfx_dalpha: Partial derivative of fx/mass w.r.t. alpha (NaN for no solution)
        dfz_dalpha: Partial derivative of fz/mass w.r.t. alpha (NaN for no solution)
        residuals: Residual error of trim equation (NaN for velocities with no solution)
        aero_type: Aerodynamic type identifier
        mass: Aircraft mass in kg
    """
    velocities: np.ndarray
    alphas: np.ndarray
    cx: np.ndarray
    cz: np.ndarray
    fx: np.ndarray
    fz: np.ndarray
    dfx_dalpha: np.ndarray
    dfz_dalpha: np.ndarray
    residuals: np.ndarray
    aero_type: str
    mass: float


def compute_trim_two_branches(
    velocities: np.ndarray,
    aero_type: str,
    mass: float = 2.0,
    g: float = 9.81,
) -> TrimResults:
    """Compute trim conditions capturing both solution branches with simple threshold merge.

    Strategy:
    1. Run descending (high→low velocity) to capture low-AoA branch
    2. Run ascending (low→high velocity) to capture high-AoA branch
    3. Merge using velocity threshold: high-AoA for V < 11, low-AoA for V >= 11

    Args:
        velocities: Array of velocities to solve for (m/s)
        aero_type: Aerodynamic type ("lyu", "bspline", "phi", "advanced")
        mass: Aircraft mass in kg (default: 2.0)
        g: Gravity in m/s² (default: 9.81)

    Returns:
        TrimResults containing both solution branches merged at threshold.
    """
    # Pass 1: Descending (high→low) with low initial guess
    print("  Pass 1: Descending velocity (low-AoA branch)...")
    results_low = compute_trim_continuation(
        velocities, aero_type, mass, g,
        alpha_initial_guess_deg=10.0,
        direction="descending",
        max_delta_alpha_deg=5.0,
    )

    # Pass 2: Ascending (low→high) with high initial guess
    print("  Pass 2: Ascending velocity (high-AoA branch)...")
    results_high = compute_trim_continuation(
        velocities, aero_type, mass, g,
        alpha_initial_guess_deg=70.0,
        direction="ascending",
        max_delta_alpha_deg=5.0,
    )

    # Smart merge: start from low velocity and always pick closest to previous solution
    n_points = len(velocities)
    alphas_merged = np.full(n_points, np.nan)
    cx_vals_merged = np.full(n_points, np.nan)
    cz_vals_merged = np.full(n_points, np.nan)
    fx_vals_merged = np.full(n_points, np.nan)
    fz_vals_merged = np.full(n_points, np.nan)
    dfx_dalpha_vals_merged = np.full(n_points, np.nan)
    dfz_dalpha_vals_merged = np.full(n_points, np.nan)
    residual_vals_merged = np.full(n_points, np.nan)

    for i in range(n_points):
        has_low = not np.isnan(results_low.alphas[i])
        has_high = not np.isnan(results_high.alphas[i])

        if i == 0:
            # First point: prefer high-AoA branch (low velocity regime)
            if has_high:
                alphas_merged[i] = results_high.alphas[i]
                cx_vals_merged[i] = results_high.cx[i]
                cz_vals_merged[i] = results_high.cz[i]
                fx_vals_merged[i] = results_high.fx[i]
                fz_vals_merged[i] = results_high.fz[i]
                dfx_dalpha_vals_merged[i] = results_high.dfx_dalpha[i]
                dfz_dalpha_vals_merged[i] = results_high.dfz_dalpha[i]
                residual_vals_merged[i] = results_high.residuals[i]
            elif has_low:
                alphas_merged[i] = results_low.alphas[i]
                cx_vals_merged[i] = results_low.cx[i]
                cz_vals_merged[i] = results_low.cz[i]
                fx_vals_merged[i] = results_low.fx[i]
                fz_vals_merged[i] = results_low.fz[i]
                dfx_dalpha_vals_merged[i] = results_low.dfx_dalpha[i]
                dfz_dalpha_vals_merged[i] = results_low.dfz_dalpha[i]
                residual_vals_merged[i] = results_low.residuals[i]
        else:
            # Subsequent points: choose solution closest to previous solution
            prev_alpha = alphas_merged[i-1]

            if has_low and has_high:
                # Both branches available: choose closest to previous
                delta_low = abs(results_low.alphas[i] - prev_alpha)
                delta_high = abs(results_high.alphas[i] - prev_alpha)

                if delta_low < delta_high:
                    # Low-AoA branch maintains continuity
                    alphas_merged[i] = results_low.alphas[i]
                    cx_vals_merged[i] = results_low.cx[i]
                    cz_vals_merged[i] = results_low.cz[i]
                    fx_vals_merged[i] = results_low.fx[i]
                    fz_vals_merged[i] = results_low.fz[i]
                    dfx_dalpha_vals_merged[i] = results_low.dfx_dalpha[i]
                    dfz_dalpha_vals_merged[i] = results_low.dfz_dalpha[i]
                    residual_vals_merged[i] = results_low.residuals[i]
                else:
                    # High-AoA branch maintains continuity
                    alphas_merged[i] = results_high.alphas[i]
                    cx_vals_merged[i] = results_high.cx[i]
                    cz_vals_merged[i] = results_high.cz[i]
                    fx_vals_merged[i] = results_high.fx[i]
                    fz_vals_merged[i] = results_high.fz[i]
                    dfx_dalpha_vals_merged[i] = results_high.dfx_dalpha[i]
                    dfz_dalpha_vals_merged[i] = results_high.dfz_dalpha[i]
                    residual_vals_merged[i] = results_high.residuals[i]
            elif has_low:
                # Only low-AoA available
                alphas_merged[i] = results_low.alphas[i]
                cx_vals_merged[i] = results_low.cx[i]
                cz_vals_merged[i] = results_low.cz[i]
                fx_vals_merged[i] = results_low.fx[i]
                fz_vals_merged[i] = results_low.fz[i]
                dfx_dalpha_vals_merged[i] = results_low.dfx_dalpha[i]
                dfz_dalpha_vals_merged[i] = results_low.dfz_dalpha[i]
                residual_vals_merged[i] = results_low.residuals[i]
            elif has_high:
                # Only high-AoA available
                alphas_merged[i] = results_high.alphas[i]
                cx_vals_merged[i] = results_high.cx[i]
                cz_vals_merged[i] = results_high.cz[i]
                fx_vals_merged[i] = results_high.fx[i]
                fz_vals_merged[i] = results_high.fz[i]
                dfx_dalpha_vals_merged[i] = results_high.dfx_dalpha[i]
                dfz_dalpha_vals_merged[i] = results_high.dfz_dalpha[i]
                residual_vals_merged[i] = results_high.residuals[i]

    return TrimResults(
        velocities=velocities,
        alphas=alphas_merged,
        cx=cx_vals_merged,
        cz=cz_vals_merged,
        fx=fx_vals_merged,
        fz=fz_vals_merged,
        dfx_dalpha=dfx_dalpha_vals_merged,
        dfz_dalpha=dfz_dalpha_vals_merged,
        residuals=residual_vals_merged,
        aero_type=aero_type,
        mass=mass,
    )


def compute_trim_continuation(
    velocities: np.ndarray,
    aero_type: str,
    mass: float = 2.0,
    g: float = 9.81,
    alpha_initial_guess_deg: float = 50.0,
    direction: str = "descending",  # "descending" or "ascending"
    max_delta_alpha_deg: float = 5.0,
) -> TrimResults:
    """Compute trim conditions using brentq root-finding with continuation method.

    For each velocity, solves the 1D root-finding problem with penalty term:
        cz(α) * V² / mass + g * cos(α) + λ * |α - α_mean|² = 0

    The penalty term λ * |α - α_mean|² naturally selects one solution branch
    by penalizing deviations from the running mean of previous solutions.

    Args:
        velocities: Array of velocities to solve for (m/s)
        aero_type: Aerodynamic type ("lyu", "bspline", "phi", "advanced")
        mass: Aircraft mass in kg (default: 2.0)
        g: Gravity in m/s² (default: 9.81)
        alpha_initial_guess_deg: Initial angle of attack guess in degrees (default: 50.0)
        direction: Velocity traversal direction - "descending" (high→low) or "ascending" (low→high)
        max_delta_alpha_deg: Maximum allowed alpha change between steps to stay on same branch (default: 5.0)
        penalty_weight: Weight for penalty term (default: 10.0)

    Returns:
        TrimResults containing solutions for all velocities.
        Velocities with no solution have NaN values.
    """
    # Load predictor and derivatives
    predictor = load_aerodynamics_mlp(aero_type)
    derivatives = load_aero_derivatives(aero_type)

    # Initialize result arrays with NaN (no solution)
    n_points = len(velocities)
    alphas = np.full(n_points, np.nan)
    cx_vals = np.full(n_points, np.nan)
    cz_vals = np.full(n_points, np.nan)
    fx_vals = np.full(n_points, np.nan)
    fz_vals = np.full(n_points, np.nan)
    dfx_dalpha_vals = np.full(n_points, np.nan)
    dfz_dalpha_vals = np.full(n_points, np.nan)
    residual_vals = np.full(n_points, np.nan)

    # Initial guess for alpha (start with moderate AoA)
    alpha_guess = np.deg2rad(alpha_initial_guess_deg)

    # AoA constraint: only accept solutions in [0, 90 degrees]
    alpha_min = 0.0
    alpha_max = np.deg2rad(90.0)

    # Penalty term parameters
    penalty_weight = 0.5  # Weight for penalty term
    alpha_history = []  # Track valid solutions for computing mean
    alpha_mean = alpha_guess  # Running mean of valid solutions

    # Continuation loop: iterate through velocities in specified direction
    if direction == "descending":
        # HIGH to LOW velocity - good for finding low-AoA branch
        indices = np.argsort(-velocities)  # Sort by descending velocity
    else:  # ascending
        # LOW to HIGH velocity - good for finding high-AoA branch
        indices = np.argsort(velocities)  # Sort by ascending velocity

    for idx in indices:
        V = velocities[idx]
        i = idx  # Original index for storing results

        # Skip V=0 (singular)
        if V < 0.1:
            continue

        # Define residual function for this velocity with penalty term
        def residual(
            alpha: float,
            V: float = V,
            alpha_mean: float = alpha_mean,
        ) -> float:
            cx_jax, cz_jax = predictor.predict(alpha)  # Returns (cx, cz)
            cz_val = float(cz_jax.item() if hasattr(cz_jax, 'item') else cz_jax)
            # Add penalty term to favor one branch
            penalty = penalty_weight * (alpha - alpha_mean)**2
            return cz_val * V**2 / mass + g * np.cos(alpha) + penalty

        # Try to solve using brentq in [0, 90 deg] ONLY
        try:
            # Check if bracket is valid
            f_min = residual(alpha_min)
            f_max = residual(alpha_max)

            # Try to find a valid bracket in [0, 90 deg]
            bracket = None
            if f_min * f_max < 0:
                # Direct bracket works
                bracket = [alpha_min, alpha_max]
            else:
                # Try using alpha_guess as one endpoint to find sub-range
                f_guess = residual(alpha_guess)
                if f_min * f_guess < 0:
                    bracket = [alpha_min, alpha_guess]
                elif f_guess * f_max < 0:
                    bracket = [alpha_guess, alpha_max]

            if bracket is None:
                # No valid bracket in [0, 90 deg] - no solution for this velocity
                continue

            result = root_scalar(
                residual,
                bracket=bracket,
                method='brentq',
                xtol=1e-8,
                rtol=1e-8,
            )

            if result.converged:
                alpha_sol = result.root

                # Verify solution is in [0, 90 deg]
                if alpha_sol < alpha_min or alpha_sol > alpha_max:
                    # Solution outside valid range - skip
                    continue

                # Compute residual at solution
                residual_val = residual(alpha_sol)
                residual_vals[i] = residual_val

                # Check if residual is small enough (tolerance: 1e-3)
                if abs(residual_val) >= 1e-3:
                    # Residual too large - skip
                    continue

                # Valid solution found in [0, 90 deg]
                alphas[i] = alpha_sol

                # Update running mean for penalty term
                alpha_history.append(alpha_sol)
                alpha_mean = np.mean(alpha_history)

                # Compute all outputs at solution
                cx_jax, cz_jax = predictor.predict(alpha_sol)
                cx_vals[i] = float(cx_jax.item() if hasattr(cx_jax, 'item') else cx_jax)
                cz_vals[i] = float(cz_jax.item() if hasattr(cz_jax, 'item') else cz_jax)

                fx_vals[i] = cx_vals[i] * V**2 / mass
                fz_vals[i] = cz_vals[i] * V**2 / mass

                # Compute derivatives
                dcx, dcz = derivatives.compute_jacobian(alpha_sol)
                dfx_dalpha_vals[i] = V**2 / mass * float(dcx.item() if hasattr(dcx, 'item') else dcx)
                dfz_dalpha_vals[i] = V**2 / mass * float(dcz.item() if hasattr(dcz, 'item') else dcz)

                # Update initial guess for next iteration (continuation)
                alpha_guess = alpha_sol

        except (ValueError, RuntimeError):
            # Root finding failed - leave values as NaN
            continue

    return TrimResults(
        velocities=velocities,
        alphas=alphas,
        cx=cx_vals,
        cz=cz_vals,
        fx=fx_vals,
        fz=fz_vals,
        dfx_dalpha=dfx_dalpha_vals,
        dfz_dalpha=dfz_dalpha_vals,
        residuals=residual_vals,
        aero_type=aero_type,
        mass=mass,
    )


def run_trim_analysis_all_types(
    velocities: np.ndarray,
    mass: float = 2.0,
    g: float = 9.81,
) -> dict[str, TrimResults]:
    """Run trim analysis for all 4 aerodynamic types.

    Args:
        velocities: Array of velocities to solve for (m/s)
        mass: Aircraft mass in kg (default: 2.0)
        g: Gravity in m/s² (default: 9.81)

    Returns:
        Dictionary mapping aero_type -> TrimResults
    """
    aero_types = ["lyu", "bspline", "phi"]
    results = {}

    for aero_type in aero_types:
        print(f"Computing trim for {aero_type}...")
        try:
            results[aero_type] = compute_trim_two_branches(
                velocities, aero_type, mass, g
            )
            n_valid = np.sum(~np.isnan(results[aero_type].alphas))
            print(f"  Converged: {n_valid}/{len(velocities)} points")
        except Exception as e:
            print(f"  Error computing trim for {aero_type}: {e}")
            results[aero_type] = None

    return results


def save_trim_results_to_csv(
    results: TrimResults,
    output_dir: Path = Path(".artifacts/aerodynamics"),
) -> Path:
    """Save trim results to CSV file.

    Args:
        results: TrimResults to save
        output_dir: Output directory path (project-relative; default .artifacts/aerodynamics)

    Returns:
        Path to the saved CSV file
    """
    output_dir = artifact_path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    filename = f"trim_analysis_{results.aero_type}_mass{results.mass}kg.csv"
    filepath = output_dir / filename

    df = pd.DataFrame({
        "velocity_ms": results.velocities,
        "alpha_deg": np.rad2deg(results.alphas),
        "cx": results.cx,
        "cz": results.cz,
        "fx_per_mass": results.fx,
        "fz_per_mass": results.fz,
        "dfx_dalpha": results.dfx_dalpha,
        "dfz_dalpha": results.dfz_dalpha,
        "residual": results.residuals,
    })

    df.to_csv(filepath, index=False)
    print(f"  Saved CSV: {filepath}")

    return filepath


def plot_trim_results_2x2(
    results: TrimResults,
    output_path: Path | None = None,
    show: bool = False,
) -> None:
    """Create 2x2 subplot for trim analysis results.

    Args:
        results: TrimResults to plot
        output_path: Path to save plot (None to skip saving)
        show: Whether to display plot interactively (default: False)
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f"Trim Analysis: {results.aero_type.upper()} (Mass={results.mass}kg)",
        fontsize=16,
    )

    # Filter out NaN values for plotting
    valid_mask = ~np.isnan(results.alphas)
    alpha_deg = np.rad2deg(results.alphas[valid_mask])

    if np.sum(valid_mask) == 0:
        print(f"  Warning: No valid data to plot for {results.aero_type}")
        return

    # Plot 1: cx vs AoA
    ax1 = axes[0, 0]
    ax1.plot(alpha_deg, results.cx[valid_mask], "b-", linewidth=2)
    ax1.set_xlabel("Angle of Attack (degrees)")
    ax1.set_ylabel("cx")
    ax1.set_title("Drag Coefficient vs AoA")
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 90])

    # Plot 2: cz vs AoA
    ax2 = axes[0, 1]
    ax2.plot(alpha_deg, results.cz[valid_mask], "r-", linewidth=2)
    ax2.set_xlabel("Angle of Attack (degrees)")
    ax2.set_ylabel("cz")
    ax2.set_title("Lift Coefficient vs AoA")
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, 90])

    # Plot 3: ∂fx/mass w.r.t. alpha vs AoA
    ax3 = axes[1, 0]
    ax3.plot(alpha_deg, results.dfx_dalpha[valid_mask], "g-", linewidth=2)
    ax3.set_xlabel("Angle of Attack (degrees)")
    ax3.set_ylabel("∂fx/mass ∂α")
    ax3.set_title("X-Force Derivative vs AoA")
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim([0, 90])

    # Plot 4: ∂fz/mass w.r.t. alpha vs AoA
    ax4 = axes[1, 1]
    ax4.plot(alpha_deg, results.dfz_dalpha[valid_mask], "m-", linewidth=2)
    ax4.set_xlabel("Angle of Attack (degrees)")
    ax4.set_ylabel("∂fz/mass ∂α")
    ax4.set_title("Z-Force Derivative vs AoA")
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim([0, 90])

    plt.tight_layout()

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"  Saved plot: {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def plot_all_trim_results(
    results_dict: dict[str, TrimResults],
    output_dir: Path = Path(".artifacts/aerodynamics"),
    show: bool = False,
) -> list[Path]:
    """Generate plots for all aerodynamic types.

    Args:
        results_dict: Dictionary mapping aero_type -> TrimResults
        output_dir: Output directory path (project-relative; default .artifacts/aerodynamics)
        show: Whether to display plots interactively (default: False)

    Returns:
        List of paths to saved plot files
    """
    output_dir = artifact_path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    saved_paths = []

    for aero_type, results in results_dict.items():
        if results is None:
            continue

        filename = f"trim_analysis_{aero_type}_mass{results.mass}kg.png"
        output_path = output_dir / filename

        plot_trim_results_2x2(results, output_path=output_path, show=show)
        saved_paths.append(output_path)

    return saved_paths


def plot_trim_comparison_2x2(
    results_dict: dict[str, TrimResults],
    output_path: Path | None = None,
    show: bool = False,
) -> None:
    """Create 2x2 comparison plot for all aerodynamic types.

    Args:
        results_dict: Dictionary mapping aero_type -> TrimResults
        output_path: Path to save plot (None to skip saving)
        show: Whether to display plot interactively (default: False)
    """
    # Color and style mapping for each aero type
    styles = {
        "lyu": {"color": "blue", "marker": "o", "linestyle": "-", "label": "Lyu"},
        "bspline": {"color": "red", "marker": "s", "linestyle": "--", "label": "BSpline"},
        "phi": {"color": "green", "marker": "^", "linestyle": "-.", "label": "Phi"},
        "advanced": {"color": "purple", "marker": "d", "linestyle": ":", "label": "Advanced"},
    }

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Trim Analysis Comparison (All Aerodynamic Types)", fontsize=16)

    # Plot each aero type
    for aero_type, results in results_dict.items():
        if results is None:
            continue

        style = styles.get(aero_type, {"color": "gray", "marker": "o", "linestyle": "-", "label": aero_type})
        valid_mask = ~np.isnan(results.alphas)

        if np.sum(valid_mask) == 0:
            continue

        alpha_deg = np.rad2deg(results.alphas[valid_mask])

        # Plot 1: cx vs AoA
        ax1 = axes[0, 0]
        ax1.plot(alpha_deg, results.cx[valid_mask], color=style["color"],
                linestyle=style["linestyle"], marker=style["marker"],
                markersize=3, markevery=max(1, len(alpha_deg) // 50),
                label=style["label"], linewidth=1.5, alpha=0.8)

        # Plot 2: cz vs AoA
        ax2 = axes[0, 1]
        ax2.plot(alpha_deg, results.cz[valid_mask], color=style["color"],
                linestyle=style["linestyle"], marker=style["marker"],
                markersize=3, markevery=max(1, len(alpha_deg) // 50),
                label=style["label"], linewidth=1.5, alpha=0.8)

        # Plot 3: ∂fx/mass w.r.t. alpha vs AoA
        ax3 = axes[1, 0]
        ax3.plot(alpha_deg, results.dfx_dalpha[valid_mask], color=style["color"],
                linestyle=style["linestyle"], marker=style["marker"],
                markersize=3, markevery=max(1, len(alpha_deg) // 50),
                label=style["label"], linewidth=1.5, alpha=0.8)

        # Plot 4: ∂fz/mass w.r.t. alpha vs AoA
        ax4 = axes[1, 1]
        ax4.plot(alpha_deg, results.dfz_dalpha[valid_mask], color=style["color"],
                linestyle=style["linestyle"], marker=style["marker"],
                markersize=3, markevery=max(1, len(alpha_deg) // 50),
                label=style["label"], linewidth=1.5, alpha=0.8)

    # Configure axes
    for ax, (ylabel, title) in zip(
        axes.flat,
        [
            ("cx", "Drag Coefficient vs AoA"),
            ("cz", "Lift Coefficient vs AoA"),
            ("∂fx/mass ∂α", "X-Force Derivative vs AoA"),
            ("∂fz/mass ∂α", "Z-Force Derivative vs AoA"),
        ], strict=False,
    ):
        ax.set_xlabel("Angle of Attack (degrees)")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.set_xlim([0, 90])
        ax.legend(loc="lower right", fontsize=8, ncol=3)

    plt.tight_layout()

    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"  Saved comparison plot: {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def plot_derivative_comparison_2x1(
    results_dict: dict[str, TrimResults],
    output_path: Path | None = None,
    show: bool = False,
) -> None:
    """Create 2x1 comparison plot for partial derivatives only (PGF format).

    Args:
        results_dict: Dictionary mapping aero_type -> TrimResults
        output_path: Path to save plot (None to skip saving)
        show: Whether to display plot interactively (default: False)
    """
    import matplotlib as mpl

    # Configure PGF backend with 8pt font
    mpl.use("pgf")
    mpl.rcParams.update({
        "pgf.texsystem": "pdflatex",
        "text.usetex": True,
        "font.family": "serif",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "figure.titlesize": 8,
        "pgf.preamble": r"\usepackage{amsmath}",
    })

    # Color and style mapping for each aero type
    styles = {
        "lyu": {"color": "blue", "marker": "o", "linestyle": "-", "label": r"Lyu \textit{et al.}"},
        "bspline": {"color": "red", "marker": "s", "linestyle": ":", "label": r"Ma \textit{et al.}"},
        "phi": {"color": "green", "marker": "^", "linestyle": "-", "label": r"$\phi$-theory"},
        "advanced": {"color": "purple", "marker": "d", "linestyle": ":", "label": r"Gazebo"},
    }

    # Figure size: 3.5 inches width, 2.0 inches height
    # Height ratio: axes[0] = 2/5, axes[1] = 3/5
    from matplotlib.gridspec import GridSpec
    fig = plt.figure(figsize=(3.5, 1.5))
    gs = GridSpec(2, 1, figure=fig, height_ratios=[1, 1], hspace=0.5)
    axes = [fig.add_subplot(gs[0]), fig.add_subplot(gs[1])]

    # Plot each aero type
    for aero_type, results in results_dict.items():
        if results is None:
            continue

        style = styles.get(aero_type, {"color": "gray", "marker": "o", "linestyle": "-", "label": aero_type})
        valid_mask = ~np.isnan(results.alphas)

        if np.sum(valid_mask) == 0:
            continue

        alpha_deg = np.rad2deg(results.alphas[valid_mask])

        # Scale derivatives by 1/9.8
        scale_factor = 1.0 / 9.8

        # Plot 1: ∂fx/mass w.r.t. alpha vs AoA (scaled)
        ax1 = axes[0]
        ax1.plot(alpha_deg, results.dfx_dalpha[valid_mask] * scale_factor, color=style["color"],
                linestyle=style["linestyle"], marker=style["marker"],
                markersize=2, markevery=max(1, len(alpha_deg) // 50),
                label=style["label"], linewidth=1.0, alpha=0.8)

        ax1.set_ylabel(r"$\frac{\partial\,^{\mathcal{B}}\!\boldsymbol{f}_{a_x}}{\partial \alpha} [mg]$")
        # Plot 2: ∂fz/mass w.r.t. alpha vs AoA (scaled)
        ax2 = axes[1]
        ax2.plot(alpha_deg, results.dfz_dalpha[valid_mask] * scale_factor, color=style["color"],
                linestyle=style["linestyle"], marker=style["marker"],
                markersize=2, markevery=max(1, len(alpha_deg) // 50),
                label=style["label"], linewidth=1.0, alpha=0.8)
        ax2.set_ylabel(r"$\frac{\partial\,^{\mathcal{B}}\!\boldsymbol{f}_{a_z}}{\partial \alpha} [mg]$")

    # Configure axes
    for ax in axes:
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.set_xlim([0, 90])
        # Set ticks to integers
        ax.set_xticks(range(0, 91, 30))
        ax.set_yticks([int(y) for y in ax.get_yticks() if int(y) == y])
        # Remove top and right spines
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    axes[1].set_xlabel(r"$\alpha$ [deg]")
    # Remove y-labels (keep ticks), legend: 2x2 layout with 6pt
    axes[1].legend(loc="lower right", fontsize=6, framealpha=0.8, ncol=3, columnspacing=0.5, handletextpad=0.3)

    if output_path:
        output_path = Path(output_path)
        # Change extension to .pdf
        if output_path.suffix == ".png":
            output_path = output_path.with_suffix(".pdf")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, format="pdf", backend="pgf", bbox_inches="tight")
        print(f"  Saved derivative comparison plot (PDF via PGF): {output_path}")

    if show:
        plt.show()
    else:
        plt.close()

    # Reset to default backend
    mpl.use("Agg")


def main() -> None:
    """Main entry point for command-line usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Neural Network-Based Trim Analysis Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mass",
        type=float,
        default=2.0,
        help="Aircraft mass in kg (default: 2.0)",
    )
    parser.add_argument(
        "--v-max",
        type=float,
        default=14.0,
        help="Maximum velocity in m/s (default: 14.0)",
    )
    parser.add_argument(
        "--v-min",
        type=float,
        default=1.0,
        help="Minimum velocity in m/s (default: 0.1, must be > 0)",
    )
    parser.add_argument(
        "--num-points",
        type=int,
        default=300,
        help="Number of velocity points (default: 300)",
    )
    parser.add_argument(
        "--aero-type",
        type=str,
        choices=["lyu", "bspline", "phi", "advanced", "all"],
        default="all",
        help="Aerodynamic type (default: all)",
    )
    parser.add_argument(
        "--plot-only",
        action="store_true",
        help="Only generate plots from existing CSV files",
    )
    parser.add_argument(
        "--show-plots",
        action="store_true",
        help="Display plots interactively",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=".artifacts/aerodynamics",
        help="Output directory for results (default: .artifacts/aerodynamics)",
    )

    args = parser.parse_args()

    output_dir = artifact_path(Path(args.output_dir))

    if args.plot_only:
        # Load existing CSV files and generate plots
        print("Generating plots from existing CSV files...")
        aero_types = ["lyu", "bspline", "phi"]
        results_dict = {}

        for aero_type in aero_types:
            csv_path = output_dir / f"trim_analysis_{aero_type}_mass{args.mass}kg.csv"
            if csv_path.exists():
                df = pd.read_csv(csv_path)
                # Handle both old CSV (no residual) and new CSV (with residual)
                if "residual" in df.columns:
                    residuals = df["residual"].values
                else:
                    residuals = np.full(len(df), np.nan)
                results = TrimResults(
                    velocities=df["velocity_ms"].values,
                    alphas=np.deg2rad(df["alpha_deg"].values),
                    cx=df["cx"].values,
                    cz=df["cz"].values,
                    fx=df["fx_per_mass"].values,
                    fz=df["fz_per_mass"].values,
                    dfx_dalpha=df["dfx_dalpha"].values,
                    dfz_dalpha=df["dfz_dalpha"].values,
                    residuals=residuals,
                    aero_type=aero_type,
                    mass=args.mass,
                )
                results_dict[aero_type] = results
                plot_path = output_dir / f"trim_analysis_{aero_type}_mass{args.mass}kg.png"
                plot_trim_results_2x2(results, output_path=plot_path, show=args.show_plots)

        # Generate comparison plots
        if len(results_dict) > 1:
            print("\nGenerating comparison plots...")
            comparison_path = output_dir / f"trim_comparison_mass{args.mass}kg.png"
            plot_trim_comparison_2x2(results_dict, output_path=comparison_path, show=args.show_plots)

            derivative_path = output_dir / f"derivative_comparison_mass{args.mass}kg.pdf"
            plot_derivative_comparison_2x1(results_dict, output_path=derivative_path, show=args.show_plots)

        return

    # Run trim analysis
    print("=" * 60)
    print("Neural Network-Based Trim Analysis")
    print("=" * 60)
    print(f"Mass: {args.mass} kg")
    print(f"Velocity range: [{args.v_min}, {args.v_max}] m/s")
    print(f"Number of points: {args.num_points}")
    print(f"Aerodynamic type: {args.aero_type}")
    print("=" * 60)

    # Generate velocity array
    velocities = np.linspace(args.v_min, args.v_max, args.num_points)

    # Determine which aero types to process
    if args.aero_type == "all":
        aero_types = ["lyu", "bspline", "phi"]
    else:
        aero_types = [args.aero_type]

    # Run analysis
    results_dict = {}
    for aero_type in aero_types:
        print(f"\nComputing trim for {aero_type}...")
        try:
            results = compute_trim_two_branches(
                velocities, aero_type, args.mass
            )
            results_dict[aero_type] = results

            # Save CSV
            save_trim_results_to_csv(results, output_dir=output_dir)

            # Generate individual plot
            plot_path = output_dir / f"trim_analysis_{aero_type}_mass{args.mass}kg.png"
            plot_trim_results_2x2(results, output_path=plot_path, show=args.show_plots)

        except Exception as e:
            print(f"  Error: {e}")
            import traceback
            traceback.print_exc()

    # Generate comparison plot (only if we have multiple aero types)
    if len(results_dict) > 1:
        print("\nGenerating comparison plots...")
        comparison_path = output_dir / f"trim_comparison_mass{args.mass}kg.png"
        plot_trim_comparison_2x2(results_dict, output_path=comparison_path, show=args.show_plots)

        derivative_path = output_dir / f"derivative_comparison_mass{args.mass}kg.png"
        plot_derivative_comparison_2x1(results_dict, output_path=derivative_path, show=args.show_plots)

    print("\n" + "=" * 60)
    print("Analysis complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
