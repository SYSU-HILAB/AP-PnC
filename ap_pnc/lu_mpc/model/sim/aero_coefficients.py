"""
Shared aerodynamic coefficient data for all GD model backends.

The coefficient tables were historically duplicated between the JAX and CasADi
implementations which made updates error-prone.  The values now live in
``aero_coefficients.json`` and can be loaded either as NumPy or JAX arrays.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np

CoeffConverter = Callable[[Sequence[float]], Any]


@dataclass(frozen=True)
class AeroCoefficientSet:
    """Container for aerodynamic polynomial coefficients."""

    kCL: Any
    kCD: Any
    kCY: Any
    kCLL: Any
    kCm: Any
    kCn: Any


def _coefficients_path() -> Path:
    """Return the absolute path to the coefficient JSON file."""
    return Path(__file__).with_name("aero_coefficients.json")


@lru_cache(maxsize=1)
def _load_raw_coefficients() -> dict[str, list[float]]:
    """Load the coefficient dictionary from disk once."""
    coeff_path = _coefficients_path()
    with coeff_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _build_set(converter: CoeffConverter) -> AeroCoefficientSet:
    """Build a coefficient container using the provided converter."""
    raw = _load_raw_coefficients()
    return AeroCoefficientSet(
        kCL=converter(raw["kCL"]),
        kCD=converter(raw["kCD"]),
        kCY=converter(raw["kCY"]),
        kCLL=converter(raw["kCLL"]),
        kCm=converter(raw["kCm"]),
        kCn=converter(raw["kCn"]),
    )


def get_numpy_coefficients(dtype=np.float64) -> AeroCoefficientSet:
    """
    Return aerodynamic coefficients as NumPy arrays.

    Args:
        dtype: Desired NumPy dtype (defaults to float64 for numerical stability).
    """
    return _build_set(lambda values: np.array(values, dtype=dtype))


def get_jax_coefficients(dtype=None) -> AeroCoefficientSet:
    """
    Return aerodynamic coefficients as JAX arrays.

    Args:
        dtype: Optional dtype. Defaults to ``jnp.float32`` to match existing code.
    """
    import jax.numpy as jnp

    jax_dtype = dtype or jnp.float32
    return _build_set(lambda values: jnp.array(values, dtype=jax_dtype))


def get_torch_coefficients(dtype=None, device=None) -> AeroCoefficientSet:
    """
    Return aerodynamic coefficients as Torch tensors.

    Args:
        dtype: Optional torch dtype (defaults to ``torch.float32``).
        device: Optional device to place the tensors on.
    """
    import torch

    torch_dtype = dtype or torch.float32

    def _converter(values: Sequence[float]) -> torch.Tensor:
        tensor = torch.tensor(values, dtype=torch_dtype)
        return tensor.to(device) if device is not None else tensor

    return _build_set(_converter)
