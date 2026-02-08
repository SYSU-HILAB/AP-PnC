"""Inference module for aerodynamics MLP."""

from feature.aerodynamics.inference.predictor import (
    AerodynamicsMLPPredictor,
    load_aerodynamics_mlp,
    predict_coefficients,
)

__all__ = [
    "AerodynamicsMLPPredictor",
    "load_aerodynamics_mlp",
    "predict_coefficients",
]
