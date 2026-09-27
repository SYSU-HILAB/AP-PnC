"""
Plotting module for lu_mpc feature.

This module provides plotting functionality for aerodynamic coefficients, mesh visualizations,
and trajectory time-series data. All plotting functions are separated from data generation
to enable clean pipeline workflows.
"""

from .aero_plots import plot_all_aerodynamic_coeffs, plot_cl_cd_cm_coordinated
from .mesh_plots import (
    plot_aerodynamic_mesh,
    plot_aerodynamic_mesh_jax,
    plot_aerodynamic_mesh_torch,
)

__all__ = [
    "plot_cl_cd_cm_coordinated",
    "plot_all_aerodynamic_coeffs",
    "plot_aerodynamic_mesh",
    "plot_aerodynamic_mesh_jax",
    "plot_aerodynamic_mesh_torch",
]
