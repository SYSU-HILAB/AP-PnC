"""One-time import of alpha/(thrust-axis, normal-axis) coefficient artifacts.

The angle input is unchanged. Swap the two force outputs once at import, never
inside inference. Source files are preserved; destinations must be new artifacts.
"""

import argparse
import hashlib
from pathlib import Path

import numpy as np
import pandas as pd

from aerodynamics.frames import FRAME_CONVENTION
from tooling.env import artifact_path, project_path


def _paths(source, destination):
    source, destination = project_path(source), artifact_path(destination)
    if not source.is_file():
        raise FileNotFoundError(source)
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    return source, destination


def import_thrust_normal_csv(source: Path, destination: Path) -> Path:
    source, destination = _paths(source, destination)
    data = pd.read_csv(source)
    if "frame_convention" in data:
        raise ValueError("source is already frame-stamped; refusing a second axis import")
    if not {"cx", "cz"}.issubset(data.columns) or data.empty:
        raise ValueError("source must contain thrust/normal coefficient columns cx,cz")
    if not np.isfinite(data[["cx", "cz"]].to_numpy()).all():
        raise ValueError("nonfinite source coefficients")
    data[["cx", "cz"]] = data[["cz", "cx"]].to_numpy(copy=True)
    data["frame_convention"] = FRAME_CONVENTION
    data["source_sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
    with destination.open("x") as output:
        data.to_csv(output, index=False)
    return destination


def import_thrust_normal_checkpoint(source: Path, destination: Path, *, output_layer: str) -> Path:
    from safetensors import safe_open
    from safetensors.numpy import load_file, save

    source, destination = _paths(source, destination)
    with safe_open(str(source), framework="np") as checkpoint:
        metadata = dict(checkpoint.metadata() or {})
    if "frame_convention" in metadata:
        raise ValueError("source is already frame-stamped; refusing a second axis import")
    parameters = load_file(str(source))
    kernel, bias = output_layer + ".kernel", output_layer + ".bias"
    if kernel not in parameters or bias not in parameters:
        raise ValueError("declared output layer is absent")
    if (
        parameters[kernel].ndim != 2
        or parameters[kernel].shape[1] != 2
        or parameters[bias].shape != (2,)
    ):
        raise ValueError("declared output layer must have exactly two force coefficients")
    if not all(np.isfinite(value).all() for value in parameters.values()):
        raise ValueError("nonfinite source weights")
    parameters[kernel] = parameters[kernel][:, [1, 0]].copy()
    parameters[bias] = parameters[bias][[1, 0]].copy()
    metadata.update(
        frame_convention=FRAME_CONVENTION,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    )
    payload = save(parameters, metadata=metadata)
    with destination.open("xb") as output:
        output.write(payload)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--source-order", choices=["thrust-normal"], required=True)
    parser.add_argument(
        "--output-layer", help="explicit final layer key for a safetensors checkpoint"
    )
    args = parser.parse_args()
    if args.output_layer is None:
        print(import_thrust_normal_csv(args.source, args.destination))
    else:
        print(
            import_thrust_normal_checkpoint(
                args.source, args.destination, output_layer=args.output_layer
            )
        )


if __name__ == "__main__":
    main()
