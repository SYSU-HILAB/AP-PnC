"""
Safetensors checkpoint utilities.
"""

from collections.abc import Mapping
from pathlib import Path

# Type alias for FrozenDict from flax
from flax.core import FrozenDict
from safetensors import safe_open
from safetensors.flax import save_file
from safetensors.numpy import load_file

from aerodynamics.frames import FRAME_CONVENTION, require_frame
from tooling.env import artifact_path, project_path


def save_safetensors(
    params: FrozenDict,
    path: str | Path,
    metadata: dict | None = None,
) -> None:
    """
    Save model parameters to safetensors format.

    Args:
        params: Flax FrozenDict of model parameters
        path: Output file path
        metadata: Optional metadata dictionary
    """
    path = artifact_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    # Convert FrozenDict to regular dict for safetensors
    params_dict = {}
    _flatten_frozen_dict(params, params_dict)

    metadata = dict(metadata or {})
    require_frame(metadata.setdefault("frame_convention", FRAME_CONVENTION))
    save_file(params_dict, str(path), metadata=metadata)
    print(f"Saved checkpoint to: {path}")


def load_safetensors(path: str | Path) -> dict:
    """
    Load model parameters from safetensors format.

    Args:
        path: Input file path

    Returns:
        Dictionary of model parameters
    """
    path = project_path(path)
    if not path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {path}")

    with safe_open(str(path), framework="np") as checkpoint:
        require_frame((checkpoint.metadata() or {}).get("frame_convention"))
    params = load_file(str(path))
    print(f"Loaded checkpoint from: {path}")
    return params


def _flatten_frozen_dict(d: FrozenDict, out: dict, prefix: str = "") -> None:
    """
    Flatten FrozenDict for safetensors.

    Args:
        d: FrozenDict to flatten
        out: Output dictionary
        prefix: Key prefix
    """
    for key, value in d.items():
        new_key = f"{prefix}.{key}" if prefix else key
        if isinstance(value, Mapping):
            _flatten_frozen_dict(value, out, new_key)
        else:
            out[new_key] = value
