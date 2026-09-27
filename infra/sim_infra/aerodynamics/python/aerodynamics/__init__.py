"""
Aerodynamics ML Module.

Provides JAX/Flax MLP models for aerodynamic coefficient prediction.
Supports 4 aerodynamic types: lyu, bspline, phi, advanced.

Public API:
    - load_aerodynamics_mlp(aero_type): Load trained MLP predictor
    - predict_coefficients(aero_type, alpha): Predict (cx, cz) from alpha
"""

from aerodynamics.inference.predictor import (
    AerodynamicsMLPPredictor,
    load_aerodynamics_mlp,
    predict_coefficients,
)

__all__ = [
    "AerodynamicsMLPPredictor",
    "load_aerodynamics_mlp",
    "predict_coefficients",
]
