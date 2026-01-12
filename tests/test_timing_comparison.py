#!/usr/bin/env python3
"""
Timing comparison script for JAX vs PyTorch implementations of aerodynamic coefficient calculations.
This script runs benchmarks for both frameworks and provides performance comparison metrics.
"""

import sys
import os

# Add the feature/lu_mpc directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'feature', 'lu_mpc'))

try:
    from feature.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing
    from feature.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure the gd_model_jax.py and gd_model_torch.py files are in the feature/lu_mpc directory")
    sys.exit(1)


def compare_frameworks(mesh_size=181, num_runs=5):
    """
    Compare performance between JAX and PyTorch implementations.
    """
    print("=" * 60)
    print("AERODYNAMIC COEFFICIENT COMPUTATION TIMING COMPARISON")
    print("=" * 60)

    # Run JAX benchmark
    print("\n" + "=" * 60)
    try:
        jax_results = benchmark_jax_timing(mesh_size=mesh_size, num_runs=num_runs)
        jax_success = True
    except Exception as e:
        print(f"JAX benchmark failed: {e}")
        jax_results = None
        jax_success = False

    # Run PyTorch benchmark
    print("\n" + "=" * 60)
    try:
        torch_results = benchmark_torch_timing(mesh_size=mesh_size, num_runs=num_runs)
        torch_success = True
    except Exception as e:
        print(f"PyTorch benchmark failed: {e}")
        torch_results = None
        torch_success = False

    # Compare results if both succeeded
    if jax_success and torch_success:
        print("\n" + "=" * 60)
        print("PERFORMANCE COMPARISON SUMMARY")
        print("=" * 60)

        jax_avg = jax_results['avg_time']
        torch_avg = torch_results['avg_time']

        print(f"Mesh size: {mesh_size}x{mesh_size} ({mesh_size*mesh_size:,} points)")
        print(f"Number of runs: {num_runs}")
        print()
        print(f"JAX Results:")
        print(f"  Average time: {jax_avg:.2f} ms")
        print(f"  Min time: {jax_results['min_time']:.2f} ms")
        print(f"  Max time: {jax_results['max_time']:.2f} ms")
        print(f"  Std Dev: {jax_results['std_time']:.2f} ms")
        print(f"  Throughput: {jax_results['throughput']:.0f} points/ms")
        print()
        print(f"PyTorch Results:")
        print(f"  Average time: {torch_avg:.2f} ms")
        print(f"  Min time: {torch_results['min_time']:.2f} ms")
        print(f"  Max time: {torch_results['max_time']:.2f} ms")
        print(f"  Std Dev: {torch_results['std_time']:.2f} ms")
        print(f"  Throughput: {torch_results['throughput']:.0f} points/ms")
        print()

        # Calculate speedup
        if jax_avg < torch_avg:
            speedup = torch_avg / jax_avg
            print(f"JAX is {speedup:.2f}x faster than PyTorch")
        else:
            speedup = jax_avg / torch_avg
            print(f"PyTorch is {speedup:.2f}x faster than JAX")

        print(f"Time difference: {abs(jax_avg - torch_avg):.2f} ms")

        return {
            'jax_results': jax_results,
            'torch_results': torch_results,
            'speedup': speedup,
            'winner': 'JAX' if jax_avg < torch_avg else 'PyTorch'
        }
    else:
        print("\nCould not complete comparison due to benchmark failures.")
        return None


if __name__ == "__main__":
    # Default parameters
    mesh_size = 181  # Smaller default for faster testing (181x181 = 32,761 points)
    num_runs = 5

    # Allow command line arguments
    if len(sys.argv) > 1:
        try:
            mesh_size = int(sys.argv[1])
        except ValueError:
            print("Usage: python timing_comparison.py [mesh_size] [num_runs]")
            sys.exit(1)

    if len(sys.argv) > 2:
        try:
            num_runs = int(sys.argv[2])
        except ValueError:
            print("Usage: python timing_comparison.py [mesh_size] [num_runs]")
            sys.exit(1)

    # Run comparison
    result = compare_frameworks(mesh_size=mesh_size, num_runs=num_runs)

    if result:
        print(f"\nWinner: {result['winner']} (speedup: {result['speedup']:.2f}x)")
    else:
        print("\nComparison failed.")