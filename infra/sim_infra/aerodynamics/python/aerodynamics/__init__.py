"""Body-FLU aerodynamic data and JAX/Flax coefficient models.

Data/frame validation does not initialize an ML runtime. Predictor entry points
are loaded on demand; their force outputs are (Fx/V², Fz/V²) in body FLU.
"""

from importlib import import_module

__all__ = [
    "AerodynamicsMLPPredictor",
    "load_aerodynamics_mlp",
    "predict_coefficients",
]


def __getattr__(name):
    if name not in __all__:
        raise AttributeError(name)
    value = getattr(import_module("aerodynamics.inference.predictor"), name)
    globals()[name] = value
    return value
