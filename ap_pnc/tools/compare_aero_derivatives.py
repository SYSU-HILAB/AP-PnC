#!/usr/bin/env python
"""
Compare JAX-derived coefficients and derivatives across aerodynamic MLP models.

Generates plots with 4 subplots:
    - cx (drag coefficient) vs α
    - cz (lift coefficient) vs α
    - ∂cx/∂α (drag derivative) vs α
    - ∂cz/∂α (lift derivative) vs α

Default granularity is 1° (361 points from -π to +π).

Usage:
    uv run python ap_pnc/tools/compare_aero_derivatives.py --all
    uv run python ap_pnc/tools/compare_aero_derivatives.py --aero-type lyu
    uv run python ap_pnc/tools/compare_aero_derivatives.py --all --num-points 181 --alpha-min -0.5 --alpha-max 0.5

Note: Uses CPU by default for stability. Set JAX_PLATFORMS=gpu environment variable to use GPU.
"""

import argparse
import os

# Use CPU by default for stability (GPU has jacfwd compilation issues)
os.environ.setdefault("JAX_PLATFORMS", "cpu")

from pathlib import Path
from typing import Literal

import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from aerodynamics.inference.derivatives import AerodynamicsMLPDerivatives

AERO_TYPES = Literal["lyu", "bspline", "phi", "advanced"]


def compute_coefficient_and_derivative_stats(
    deriv: AerodynamicsMLPDerivatives, alpha_values: jnp.ndarray
) -> dict:
    """Compute statistics for coefficients and derivatives over alpha range."""
    # Get coefficients from predictor
    cx, cz = deriv.predictor.predict(alpha_values)

    # Get derivatives
    dcx_dalpha, dcz_dalpha = deriv.compute_jacobian(alpha_values)

    cx = np.array(cx)
    cz = np.array(cz)
    dcx = np.array(dcx_dalpha)
    dcz = np.array(dcz_dalpha)

    return {
        "cx": {
            "min": float(cx.min()),
            "max": float(cx.max()),
            "mean": float(cx.mean()),
            "std": float(cx.std()),
        },
        "cz": {
            "min": float(cz.min()),
            "max": float(cz.max()),
            "mean": float(cz.mean()),
            "std": float(cz.std()),
        },
        "dcx": {
            "min": float(dcx.min()),
            "max": float(dcx.max()),
            "mean": float(dcx.mean()),
            "std": float(dcx.std()),
        },
        "dcz": {
            "min": float(dcz.min()),
            "max": float(dcz.max()),
            "mean": float(dcz.mean()),
            "std": float(dcz.std()),
        },
    }


def print_statistics_table(
    results: dict[str, dict], alpha_min: float, alpha_max: float
) -> None:
    """Print formatted statistics table."""
    print(f"\n{'=' * 100}")
    print(f"Aerodynamic Coefficient Statistics for α ∈ [{alpha_min:.3f}, {alpha_max:.3f}] rad")
    print(f"{'=' * 100}\n")

    # Coefficients section
    print(f"{'Aero Type':<12} {'cx min':<12} {'cx max':<12} {'cx mean':<12}")
    print("-" * 100)

    for aero_type, stats in results.items():
        cx = stats["cx"]
        print(
            f"{aero_type:<12} {cx['min']:<12.6f} {cx['max']:<12.6f} {cx['mean']:<12.6f}"
        )

    print("\n" + "-" * 100)
    print(f"{'Aero Type':<12} {'cz min':<12} {'cz max':<12} {'cz mean':<12}")
    print("-" * 100)

    for aero_type, stats in results.items():
        cz = stats["cz"]
        print(
            f"{aero_type:<12} {cz['min']:<12.6f} {cz['max']:<12.6f} {cz['mean']:<12.6f}"
        )

    # Derivatives section
    print(f"\n{'=' * 100}")
    print(f"Derivative Statistics for α ∈ [{alpha_min:.3f}, {alpha_max:.3f}] rad")
    print(f"{'=' * 100}\n")

    print(f"{'Aero Type':<12} {'∂cx/∂α min':<12} {'∂cx/∂α max':<12} {'∂cx/∂α mean':<12}")
    print("-" * 100)

    for aero_type, stats in results.items():
        dcx = stats["dcx"]
        print(
            f"{aero_type:<12} {dcx['min']:<12.6f} {dcx['max']:<12.6f} {dcx['mean']:<12.6f}"
        )

    print("\n" + "-" * 100)
    print(f"{'Aero Type':<12} {'∂cz/∂α min':<12} {'∂cz/∂α max':<12} {'∂cz/∂α mean':<12}")
    print("-" * 100)

    for aero_type, stats in results.items():
        dcz = stats["dcz"]
        print(
            f"{aero_type:<12} {dcz['min']:<12.6f} {dcz['max']:<12.6f} {dcz['mean']:<12.6f}"
        )

    print(f"\n{'=' * 100}\n")


def plot_comparison(
    results: dict[str, dict],
    alpha_values: jnp.ndarray,
    output_path: Path,
) -> None:
    """Generate comparison plots with both coefficients and derivatives."""
    # Convert radians to degrees for x-axis
    alpha_deg = np.degrees(alpha_values)

    fig, axes = plt.subplots(4, 1, figsize=(10, 12))

    # Plot cx
    ax = axes[0]
    for aero_type, data in results.items():
        ax.plot(
            alpha_deg,
            data["cx_curve"],
            label=aero_type,
            linewidth=2,
        )
    ax.set_xlabel("α (degrees)")
    ax.set_ylabel("cx")
    ax.set_title("Drag Coefficient: cx")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Plot cz
    ax = axes[1]
    for aero_type, data in results.items():
        ax.plot(
            alpha_deg,
            data["cz_curve"],
            label=aero_type,
            linewidth=2,
        )
    ax.set_xlabel("α (degrees)")
    ax.set_ylabel("cz")
    ax.set_title("Lift Coefficient: cz")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Plot ∂cx/∂α
    ax = axes[2]
    for aero_type, data in results.items():
        ax.plot(
            alpha_deg,
            data["dcx_curve"],
            label=aero_type,
            linewidth=2,
        )
    ax.set_xlabel("α (degrees)")
    ax.set_ylabel("∂cx/∂α")
    ax.set_title("Drag Coefficient Derivative: ∂cx/∂α")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Plot ∂cz/∂α
    ax = axes[3]
    for aero_type, data in results.items():
        ax.plot(
            alpha_deg,
            data["dcz_curve"],
            label=aero_type,
            linewidth=2,
        )
    ax.set_xlabel("α (degrees)")
    ax.set_ylabel("∂cz/∂α")
    ax.set_title("Lift Coefficient Derivative: ∂cz/∂α")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Plot saved to: {output_path}")


def compare_aero_types(
    aero_types: list[AERO_TYPES],
    alpha_min: float = -jnp.pi,
    alpha_max: float = jnp.pi,
    num_points: int = 361,
    output_dir: Path | None = None,
) -> dict[str, dict]:
    """
    Compare coefficients and derivatives across multiple aerodynamic types.

    Args:
        aero_types: List of aerodynamic types to compare
        alpha_min: Minimum alpha value (radians)
        alpha_max: Maximum alpha value (radians)
        num_points: Number of points in alpha range (default: 361 for 1° granularity from -π to +π)
        output_dir: Output directory for plots (default: artifacts/simulations/)

    Returns:
        Dictionary mapping aero_type to coefficient and derivative data
    """
    if output_dir is None:
        output_dir = Path("artifacts/simulations")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate alpha values
    alpha_values = jnp.linspace(alpha_min, alpha_max, num_points)

    print("Loading models and computing coefficients and derivatives...")
    print(f"Alpha range: [{alpha_min:.3f}, {alpha_max:.3f}] ({num_points} points, ~{(alpha_max-alpha_min)/num_points:.4f} rad/point)")
    print(f"Aero types: {', '.join(aero_types)}")

    results = {}

    for aero_type in aero_types:
        print(f"\nLoading {aero_type}...", end=" ", flush=True)

        try:
            deriv = AerodynamicsMLPDerivatives.load_by_type(aero_type)

            # Compute coefficients
            cx, cz = deriv.predictor.predict(alpha_values)

            # Compute derivatives
            dcx_dalpha, dcz_dalpha = deriv.compute_jacobian(alpha_values)

            results[aero_type] = {
                "cx_curve": np.array(cx),
                "cz_curve": np.array(cz),
                "dcx_curve": np.array(dcx_dalpha),
                "dcz_curve": np.array(dcz_dalpha),
                "stats": compute_coefficient_and_derivative_stats(deriv, alpha_values),
            }

            print("✓")

        except FileNotFoundError as e:
            print(f"✗ ({e})")
            print("  Run 'uv run ap-pnc aerodyn pipeline' first to train models.")
            continue
        except Exception as e:
            print(f"✗ ({e})")
            continue

    if not results:
        print("\nNo models loaded. Cannot proceed.")
        return {}

    # Print statistics
    stats_only = {k: v["stats"] for k, v in results.items()}
    print_statistics_table(stats_only, alpha_min, alpha_max)

    # Generate plot
    output_path = output_dir / "aero_derivative_comparison.png"
    plot_comparison(results, alpha_values, output_path)

    return results


def main() -> None:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Compare JAX coefficients and derivatives for aerodynamic MLP models"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Compare all 4 aerodynamic types",
    )
    parser.add_argument(
        "--aero-type",
        type=str,
        choices=["lyu", "bspline", "phi", "advanced"],
        help="Specific aerodynamic type to analyze",
    )
    parser.add_argument(
        "--num-points",
        type=int,
        default=361,
        metavar="N",
        help="Number of points in alpha range (default: 361 for 1° granularity from -π to +π)",
    )
    parser.add_argument(
        "--alpha-min",
        type=float,
        default=-float(jnp.pi),
        help="Minimum alpha in radians (default: -π)",
    )
    parser.add_argument(
        "--alpha-max",
        type=float,
        default=float(jnp.pi),
        help="Maximum alpha in radians (default: π)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="artifacts/simulations",
        help="Output directory for plots (default: artifacts/simulations)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip generating plots",
    )

    args = parser.parse_args()

    # Determine which aero types to analyze
    if args.all:
        aero_types = ["lyu", "bspline", "phi"]
    elif args.aero_type:
        aero_types = [args.aero_type]
    else:
        parser.error("Must specify either --all or --aero-type")

    # Run comparison
    results = compare_aero_types(
        aero_types=aero_types,
        alpha_min=args.alpha_min,
        alpha_max=args.alpha_max,
        num_points=args.num_points,
        output_dir=Path(args.output_dir) if not args.no_plot else None,
    )

    if results and args.no_plot:
        print("\nSkipping plot generation (--no-plot specified).")


if __name__ == "__main__":
    main()
