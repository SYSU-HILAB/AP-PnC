#!/usr/bin/env python3
"""
Wind Performance Test Runner

This script runs automated wind performance tests for the MPC controller.
It tests tracking error and success rates under various wind conditions.

Usage:
    uv run python test/experiments/run_wind_performance_test.py

Options:
    --direct                          Skip tests, plot existing data
    --output-dir PATH                 Output directory (default: artifacts/simulations/wind_performance)
    --verbose                         Enable verbose output
    --velocities FLOAT [FLOAT ...]    Target velocities (default: 12.0)
    --side-min FLOAT                  Min side wind (default: -4.0)
    --side-max FLOAT                  Max side wind (default: 4.0)
    --side-steps INT                  Side wind steps (default: 17)
    --long-min FLOAT                  Min longitudinal wind (default: -6.0)
    --long-max FLOAT                  Max longitudinal wind (default: 6.0)
    --long-steps INT                  Longitudinal wind steps (default: 25)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

if TYPE_CHECKING:
    from ap_pnc.lu_mpc.experiments.headless_orchestrator import (
        MetricsResult,
    )


def run_single_test(
    velocity: float,
    wind_side: float,
    wind_longitudinal: float,
    trajectory_dir: str = "artifacts/simulations/temp_data",
    verbose: bool = False,
) -> MetricsResult | None:
    """
    Run a single wind performance test case.

    Args:
        velocity: Target velocity in m/s
        wind_side: Side wind component in m/s (y-axis in initial body frame)
        wind_longitudinal: Longitudinal wind component in m/s (x-axis in initial body frame)
        trajectory_dir: Directory containing trajectory CSV files
        verbose: Whether to print progress messages

    Returns:
        MetricsResult if test completed, None if test failed
    """
    from ap_pnc.lu_mpc.experiments.headless_orchestrator import HeadlessOrchestrator

    # Use existing planner trajectory (has proper aerodynamic derivatives)
    # Map test velocities to available planner trajectories
    trajectory_map: dict[float, str] = {
        12.0: "planning_line_vel12.0_thrust-2.5-2.5.csv",
    }

    if velocity in trajectory_map:
        trajectory_name = trajectory_map[velocity]
    else:
        trajectory_name = "planning_line_vel10.0_thrust-2.5-2.5.csv"

    # Build full path
    repo_root = Path(__file__).resolve().parents[2]
    csv_path = str(repo_root / trajectory_dir / trajectory_name)

    if not Path(csv_path).exists():
        if verbose:
            print(f"[ERROR] Trajectory not found: {csv_path}")
        return None

    try:
        # Run headless simulation with nmpc controller
        orchestrator = HeadlessOrchestrator(
            trajectory_csv=csv_path,
            wind_side=wind_side,
            wind_longitudinal=wind_longitudinal,
            velocity=velocity,
            controller_type="nmpc",
            verbose=verbose,
        )

        result = orchestrator.run()

        return result

    except Exception as e:
        print(f"[ERROR] Test failed: v={velocity}, ws={wind_side}, wl={wind_longitudinal}")
        print(f"  Exception: {e}")
        import traceback
        traceback.print_exc()
        return None


def main() -> int:
    """Main entry point for wind performance testing."""
    # Parse arguments
    parser = argparse.ArgumentParser(
        description="Wind performance test for MPC controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--direct",
        action="store_true",
        help="Skip data collection, load existing results and generate plots directly",
    )
    parser.add_argument(
        "--velocities",
        type=float,
        nargs="+",
        default=[12.0],
        help="Target velocities in m/s",
    )
    parser.add_argument(
        "--side-min",
        type=float,
        default=-4.0,
        help="Minimum side wind in m/s",
    )
    parser.add_argument(
        "--side-max",
        type=float,
        default=4.0,
        help="Maximum side wind in m/s",
    )
    parser.add_argument(
        "--side-steps",
        type=int,
        default=17,
        help="Number of side wind steps",
    )
    parser.add_argument(
        "--long-min",
        type=float,
        default=-6.0,
        help="Minimum longitudinal wind in m/s",
    )
    parser.add_argument(
        "--long-max",
        type=float,
        default=6.0,
        help="Maximum longitudinal wind in m/s",
    )
    parser.add_argument(
        "--long-steps",
        type=int,
        default=25,
        help="Number of longitudinal wind steps",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="artifacts/simulations/wind_performance",
        help="Output directory for results and plots",
    )
    parser.add_argument(
        "--trajectory-dir",
        type=str,
        default="artifacts/simulations/temp_data",
        help="Directory containing trajectory CSV files",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output during tests",
    )
    args = parser.parse_args()

    # Configuration
    velocities = args.velocities
    wind_side = np.linspace(args.side_min, args.side_max, args.side_steps).tolist()
    wind_longitudinal = np.linspace(args.long_min, args.long_max, args.long_steps).tolist()
    output_dir = args.output_dir
    trajectory_dir = args.trajectory_dir
    verbose = args.verbose

    # Direct mode: load existing data and generate plots
    if args.direct:
        print("=" * 70)
        print("DIRECT MODE - Loading existing data")
        print("=" * 70)

        csv_path = Path(output_dir) / "wind_performance_combined.csv"
        if not csv_path.exists():
            print(f"[ERROR] No existing data found at: {csv_path}")
            return 1

        print(f"Loading data from: {csv_path}")
        df = pd.read_csv(csv_path)

        column_mapping = {
            "Velocity [m/s]": "velocity",
            "Side Wind [m/s]": "wind_side",
            "Longitudinal Wind [m/s]": "wind_longitudinal",
            "RMS Error [m]": "rms_error",
            "MaxAE [m]": "max_error",
            "Success": "success",
        }
        df = df.rename(columns=column_mapping)

        print(f"Loaded {len(df)} test results")
        print("\nGenerating visualizations...")
        from ap_pnc.lu_mpc.plotting.wind_performance_plots import (
            generate_all_plots,
        )

        generate_all_plots(df, output_dir)
        print(f"\nResults saved to: {output_dir}")
        return 0

    # Calculate total number of tests
    num_tests = (
        len(velocities)
        * len(wind_side)
        * len(wind_longitudinal)
    )

    print("=" * 70)
    print("WIND PERFORMANCE TEST")
    print("=" * 70)
    print(f"Velocities: {velocities} m/s")
    print(f"Side Wind: {args.side_min} to {args.side_max} m/s ({args.side_steps} steps)")
    print(f"Longitudinal Wind: {args.long_min} to {args.long_max} m/s ({args.long_steps} steps)")
    print(f"Total Tests: {num_tests}")
    print(f"Output Directory: {output_dir}")
    print("=" * 70)

    # Run tests
    results = []
    start_time = time.time()

    for _i, velocity in enumerate(velocities):
        for ws in wind_side:
            for wl in wind_longitudinal:
                test_num = len(results) + 1
                print(
                    f"\n[Test {test_num}/{num_tests}] "
                    f"v={velocity:.1f}, ws={ws:.1f}, wl={wl:.1f}"
                )

                result = run_single_test(
                    velocity=velocity,
                    wind_side=ws,
                    wind_longitudinal=wl,
                    trajectory_dir=trajectory_dir,
                    verbose=verbose,
                )

                if result is not None:
                    results.append({
                        "velocity": result.velocity,
                        "wind_side": result.wind_side,
                        "wind_longitudinal": result.wind_longitudinal,
                        "rms_error": result.rms_error,
                        "max_error": result.max_error,
                        "final_error": result.final_error,
                        "success": float(result.success),
                        "num_steps": result.num_steps,
                        "trajectory_duration": result.trajectory_duration,
                    })
                    print(f"  RMS Error: {result.rms_error:.4f} m")
                    print(f"  Max Error: {result.max_error:.4f} m")
                    print(f"  Success: {result.success}")
                else:
                    print("  [FAILED] Test returned None")

    elapsed_time = time.time() - start_time

    # Print summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    print(f"Completed: {len(results)}/{num_tests} tests")
    print(f"Total Time: {elapsed_time:.1f} s ({elapsed_time/60:.1f} min)")

    if results:
        df = pd.DataFrame(results)

        success_rate = df["success"].mean() * 100
        avg_rms_error = df["rms_error"].mean()
        max_rms_error = df["rms_error"].max()

        print(f"Success Rate: {success_rate:.1f}%")
        print(f"Avg RMS Error: {avg_rms_error:.4f} m")
        print(f"Max RMS Error: {max_rms_error:.4f} m")

        # Generate plots
        print("\nGenerating visualizations...")
        from ap_pnc.lu_mpc.plotting.wind_performance_plots import (
            generate_all_plots,
        )

        generate_all_plots(df, output_dir)

        print(f"\nResults saved to: {output_dir}")

    return 0 if len(results) == num_tests else 1


if __name__ == "__main__":
    sys.exit(main())
