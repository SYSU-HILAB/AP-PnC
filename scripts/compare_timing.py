#!/usr/bin/env python3
"""
Timing comparison for the JAX and PyTorch aerodynamic coefficient implementations.
This script mirrors the old `tests/test_timing_comparison.py` behavior but now lives
under scripts/ so real tests can focus on assertions.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


from feature.lu_mpc.model.sim.gd_model_jax import benchmark_jax_timing
from feature.lu_mpc.model.sim.gd_model_torch import benchmark_torch_timing


def _format_results(label: str, results: Dict[str, float]) -> str:
    return "\n".join(
        [
            f"{label} Results:",
            f"  Average time: {results['avg_time']:.2f} ms",
            f"  Min time: {results['min_time']:.2f} ms",
            f"  Max time: {results['max_time']:.2f} ms",
            f"  Std Dev: {results['std_time']:.2f} ms",
            f"  Throughput: {results['throughput']:.0f} points/ms",
        ]
    )


def compare_frameworks(mesh_size: int, num_runs: int) -> Dict[str, object]:
    print("=" * 60)
    print("AERODYNAMIC COEFFICIENT COMPUTATION TIMING COMPARISON")
    print("=" * 60)
    print(f"Mesh size: {mesh_size}x{mesh_size} ({mesh_size * mesh_size:,} points)")
    print(f"Number of runs: {num_runs}")
    print("=" * 60)

    jax_results = benchmark_jax_timing(mesh_size=mesh_size, num_runs=num_runs)
    torch_results = benchmark_torch_timing(mesh_size=mesh_size, num_runs=num_runs)

    print("\n" + "=" * 60)
    print(_format_results("JAX", jax_results))
    print()
    print(_format_results("PyTorch", torch_results))

    winner = "JAX" if jax_results["avg_time"] <= torch_results["avg_time"] else "PyTorch"
    faster = min(jax_results["avg_time"], torch_results["avg_time"])
    slower = max(jax_results["avg_time"], torch_results["avg_time"])
    speedup = slower / faster if faster > 0 else float("inf")

    print("\n" + "=" * 60)
    print(f"Winner: {winner} (speedup: {speedup:.2f}x)")
    print(f"Time difference: {abs(jax_results['avg_time'] - torch_results['avg_time']):.2f} ms")

    return {
        "jax_results": jax_results,
        "torch_results": torch_results,
        "winner": winner,
        "speedup": speedup,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare timing between the JAX and PyTorch aerodynamic implementations."
    )
    parser.add_argument(
        "mesh_size",
        type=int,
        nargs="?",
        default=181,
        help="Number of alpha/beta samples along each axis (default: 181)",
    )
    parser.add_argument(
        "num_runs",
        type=int,
        nargs="?",
        default=5,
        help="Number of benchmark runs to average (default: 5)",
    )
    args = parser.parse_args()
    compare_frameworks(mesh_size=args.mesh_size, num_runs=args.num_runs)


if __name__ == "__main__":
    main()
