# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Erchao Rong: rongerch@outlook.com
# Zihao Liu: liuzh297@gmail.com
# Junning Liang: gordonliang27@foxmail.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

"""Regenerate the "ma" (B-spline fitted) model parameters.

Pipeline: bspline_aero_fit.py -> .artifacts/aerodynamics/bspline_fit_results/ ->
.artifacts/aerodynamics/ma.yaml. Review the result before updating the runtime
configuration in infra/sim_infra/aerodynamics/config/aero/ma.yaml.

Paths are anchored at AP_PNC_DIR (absolute), per the project path rules.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from tooling.env import artifact_path, get_git_root
from tooling.frame_contract import require_frame


class MaParamsError(Exception):
    """Raised when fit results are missing or malformed."""


@dataclass(frozen=True)
class MaParamsConfig:
    """Locations and constants for the ma parameter regeneration."""

    fit_results_dir: str = ".artifacts/aerodynamics/bspline_fit_results"
    output_yaml: str = ".artifacts/aerodynamics/ma.yaml"
    # 0.5 * rho * S / 2 * 3 * CL_slope_ref * drag_ref (see the paper).
    scale_factor: float = 0.5 * 1.184 * 0.24 / 2.0 * 3 * 0.5483863461851688 * 1.177888074834827


def _load_fit_results(root: Path) -> dict[str, np.ndarray]:
    fit_dir = artifact_path(root / MaParamsConfig().fit_results_dir)
    arrays: dict[str, np.ndarray] = {}
    for axis in ("cx", "cz"):
        path = fit_dir / f"bspline_fit_{axis}.npz"
        if not path.is_file():
            raise MaParamsError(f"missing fit results file: {path}")
        with np.load(path, allow_pickle=False) as data:
            try:
                require_frame(
                    data["frame_convention"].item() if "frame_convention" in data else None
                )
            except ValueError as exc:
                raise MaParamsError("fit results require native body-FLU coefficients") from exc
            for kind in ("coefs", "knots"):
                key = f"{axis}_{kind}"
                if key not in data.files:
                    raise MaParamsError(f"{path}: missing key '{key}'")
                arrays[key] = np.asarray(data[key], dtype=float).ravel()
    for key, values in arrays.items():
        if values.size == 0 or not np.isfinite(values).all():
            raise MaParamsError(f"{key}: empty or non-finite values")
    return arrays


def write_ma_yaml(root: Path, arrays: dict[str, np.ndarray], scale_factor: float) -> Path:
    """Stage a body-FLU configuration for review; never overwrite source files."""
    output = artifact_path(root / MaParamsConfig().output_yaml)
    output.parent.mkdir(parents=True, exist_ok=True)
    content = {
        "body_frame": "FLU",
        "scale_factor": scale_factor,
        "cx_coefs": [float(v) for v in arrays["cx_coefs"]],
        "cx_knots": [float(v) for v in arrays["cx_knots"]],
        "cz_coefs": [float(v) for v in arrays["cz_coefs"]],
        "cz_knots": [float(v) for v in arrays["cz_knots"]],
    }
    header = (
        "# Body-FLU B-spline force coefficients; alpha=atan2(vx,vz).\n"
        "# Generated under .artifacts/aerodynamics/ for review.\n"
    )
    with output.open("w", encoding="utf-8") as stream:
        stream.write(header)
        yaml.safe_dump(content, stream, sort_keys=False, default_flow_style=False)
    return output


def main() -> None:
    try:
        config = MaParamsConfig()
        root = get_git_root()
        arrays = _load_fit_results(root)
        output = write_ma_yaml(root, arrays, config.scale_factor)
        print(f"[ma-params-writer] wrote {output}")
    except MaParamsError as exc:
        print(f"Error: {exc}")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
