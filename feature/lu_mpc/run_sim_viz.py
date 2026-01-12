#!/usr/bin/env python
# -*- coding: future_fstrings -*-
#
# Simulation and Visualization Script
# Created on 2024-12-28
#
# This script runs a GD model simulation and visualizes the results.
#

import sys
import numpy as np
from feature.lu_mpc.model.sim.sim_gd_model import create_gd_simulator, GdSimConfig
from feature.lu_mpc.model.sim.utils.visualizer import plot_simulation_results


def main():
    """Run simulation and visualize results."""
    # Parse command line arguments
    sim_time = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    collect_aero = sys.argv[2].lower() == "true" if len(sys.argv) > 2 else True

    print("Running GD model simulation...")
    print(f"  Simulation time: {sim_time}s")
    print(f"  Collect aero data: {collect_aero}")

    # Create simulator
    config = GdSimConfig(dt=0.01, sim_time=sim_time)
    sim = create_gd_simulator(config)

    # Initial state: hover at 10m altitude with forward velocity
    p0 = np.array([0.0, 0.0, -10.0])  # Position (NED frame)
    v0 = np.array([5.0, 0.0, 0.0])  # Forward velocity
    r0 = np.array([0.0, 0.0, 0.0])  # Angular rates
    q0 = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion (level)
    u0 = np.array([2.4525, 2.4525, 2.4525, 2.4525])  # Motor thrusts (hover)

    # Combine initial state
    initial_state = np.concatenate([p0, v0, r0, q0, u0])

    # Control trajectory: steady hover
    N = int(sim.config.sim_time / sim.config.dt)
    control_trajectory = np.tile(u0, (N, 1))

    # Run simulation
    print(f"Running simulation with {N} steps...")
    results = sim.simulate(
        initial_state, control_trajectory, collect_aero_data=collect_aero
    )

    # Check for NaN values
    if np.any(np.isnan(results["states"])):
        print("❌ Simulation contains NaN values!")
        return 1

    print("✅ Simulation completed successfully!")
    print(f"   Final position: {results['position'][-1]}")
    print(f"   Final velocity: {results['velocity'][-1]}")

    # Get aerodynamic data
    aero_data = results.get("aero_data", None)
    if aero_data:
        print(f"   Collected {len(aero_data)} aero data points")

    # Plot results
    print("Generating plots...")
    import os
    import matplotlib

    matplotlib.use("Agg")  # Non-interactive backend

    # Create output directory if it doesn't exist
    output_dir = os.path.join(os.environ["AP_PNC_DIR"], "temp_data")
    os.makedirs(output_dir, exist_ok=True)

    # Save plots
    save_path = os.path.join(output_dir, "sim_results")
    plot_simulation_results(results, aero_data, show=False, save_path=save_path)
    print(f"✅ Plots saved to {save_path}_fig*.png")

    return 0


if __name__ == "__main__":
    sys.exit(main())
