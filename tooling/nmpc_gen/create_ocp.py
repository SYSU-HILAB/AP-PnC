# BSD 3-Clause License
# Copyright (c) 2025 Sun Yat-sen University. All rights reserved.
# Authors: Hanamy: rongerch@outlook.com

"""Generate a command-integrator OCP; numerical tuning comes only from YAML."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import yaml

from tooling.env import get_git_root

if TYPE_CHECKING:
    from acados_template import AcadosOcp


def load_nmpc_settings(path: Path | None = None) -> dict:
    """Read and validate the YAML without importing the Linux acados toolchain."""
    if path is None:
        root = get_git_root()
        path = root / "bringup/config/nmpc.yaml"
        if not path.exists():
            path = root / "core/bringup/config/nmpc.yaml"
    if not path.is_absolute():
        raise ValueError("NMPC settings path must be absolute")
    settings = yaml.safe_load(path.read_text())["nmpc"]
    required = {
        "horizon_s",
        "traj_res_s",
        "ctrl_frq",
        "initial_specific_thrust",
        "cx_alpha_slope",
        "objective",
        "command_bounds",
    }
    # Runtime-only keys are read by the controller, not baked into the OCP.
    runtime = {
        "cx_slope_estimation",
        "cx_slope_forgetting",
        "cx_slope_limit",
        "cx_slope_covariance_limit",
        "cx_slope_min_excitation",
        "cx_slope_lag_s",
    }
    if set(settings) != required | runtime:
        raise ValueError("missing or unsupported NMPC settings keys")
    for name in runtime:
        settings[name] = settings[name]
    if set(settings["objective"]) != {"position", "velocity", "body_y", "command_derivative"}:
        raise ValueError("unsupported objective keys")
    if set(settings["command_bounds"]) != {"lower", "upper"}:
        raise ValueError("unsupported command bound keys")
    for name in ("horizon_s", "traj_res_s", "ctrl_frq", "initial_specific_thrust"):
        value = float(settings[name])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"invalid NMPC {name}")
        settings[name] = value
    if 1.0 / settings["ctrl_frq"] > settings["traj_res_s"] + 1e-9:
        raise ValueError("control period must not exceed the first prediction interval")
    ratio = settings["horizon_s"] / settings["traj_res_s"]
    if not np.isclose(ratio, round(ratio), atol=1e-9, rtol=0):
        raise ValueError("NMPC horizon must contain an integer number of intervals")
    if round(ratio) < 1:
        raise ValueError("NMPC horizon must contain at least one interval")
    slope = float(settings["cx_alpha_slope"])
    # SIGNED slope: d(cx)/d(alpha). Negative for this airframe; any finite value
    # is accepted because the sign is a physical property, not a convention.
    if not np.isfinite(slope):
        raise ValueError("invalid cx alpha slope")
    settings["cx_alpha_slope"] = slope
    for name, count in (("position", 3), ("velocity", 3), ("body_y", 3), ("command_derivative", 4)):
        values = np.asarray(settings["objective"][name], dtype=float)
        if values.shape != (count,) or not np.isfinite(values).all() or (values < 0).any():
            raise ValueError(f"invalid objective {name}")
        if name == "command_derivative" and (values <= 0).any():
            raise ValueError("all command derivatives require a positive regularization weight")
    lower = np.asarray(settings["command_bounds"]["lower"], dtype=float)
    upper = np.asarray(settings["command_bounds"]["upper"], dtype=float)
    if (
        lower.shape != (4,)
        or upper.shape != (4,)
        or not np.isfinite(lower).all()
        or not np.isfinite(upper).all()
        or (lower >= upper).any()
    ):
        raise ValueError("invalid absolute command bounds")
    if not lower[0] <= settings["initial_specific_thrust"] <= upper[0]:
        raise ValueError("initial specific thrust outside command bounds")
    return settings


def ocp_nls() -> AcadosOcp:
    """Track [position, velocity, body-y]; penalize command derivatives only."""
    import casadi as cs
    from acados_template import AcadosOcp

    from tooling.nmpc_gen.model import create_tailsitter_model, rotation_matrix_from_quaternion

    settings = load_nmpc_settings()
    ocp = AcadosOcp()
    model = create_tailsitter_model()
    ocp.model = model
    rotation_world_from_body = rotation_matrix_from_quaternion(model.x[9:13])
    tracking_output = cs.vertcat(model.x[:3], model.x[3:6], rotation_world_from_body[:, 1])
    model.cost_y_expr_0 = cs.vertcat(tracking_output, model.u)
    model.cost_y_expr = model.cost_y_expr_0
    model.cost_y_expr_e = tracking_output
    ocp.cost.cost_type_0 = "NONLINEAR_LS"
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.yref_0 = np.zeros(13)
    ocp.cost.yref = np.zeros(13)
    ocp.cost.yref_e = np.zeros(9)
    ocp.parameter_values = np.zeros(1)
    assert (model.x.numel(), model.u.numel(), model.p.numel()) == (15, 4, 1)
    objective = settings["objective"]
    tracking_weights = np.concatenate([objective[k] for k in ("position", "velocity", "body_y")])
    stage_weights = np.diag(np.concatenate((tracking_weights, objective["command_derivative"])))
    # Stage 0 must penalize u_0 too; the fixed measured-state terms are constant.
    ocp.cost.W_0 = stage_weights.copy()
    ocp.cost.W = stage_weights
    ocp.cost.W_e = np.diag(tracking_weights)
    ocp.translate_nls_cost_to_conl()

    ocp.solver_options.N_horizon = round(settings["horizon_s"] / settings["traj_res_s"])
    ocp.solver_options.tf = settings["horizon_s"]
    initial_state = np.zeros(15)
    initial_state[9] = 1.0
    initial_state[13] = settings["initial_specific_thrust"]
    ocp.constraints.x0 = initial_state
    # u is dc/dt, so the old absolute u bounds must NOT be applied to it.
    # Bound the integrated absolute commands at every future node, including N.
    ocp.constraints.idxbx = np.array([13, 6, 7, 8])
    ocp.constraints.lbx = np.asarray(settings["command_bounds"]["lower"])
    ocp.constraints.ubx = np.asarray(settings["command_bounds"]["upper"])
    ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()
    ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
    ocp.constraints.ubx_e = ocp.constraints.ubx.copy()

    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_iter_max = 10
    ocp.solver_options.print_level = 0
    ocp.solver_options.hpipm_mode = "SPEED_ABS"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.exact_hess_cost = False
    return ocp


def main() -> None:
    from acados_template import AcadosOcpSolver

    ocp = ocp_nls()
    code_dir = get_git_root() / ".artifacts" / "c_generated_code"
    code_dir.mkdir(parents=True, exist_ok=True)
    ocp.code_gen_opts.code_export_directory = str(code_dir)
    AcadosOcpSolver(ocp, json_file=str(code_dir / f"acados_ocp_{ocp.model.name}.json"))


if __name__ == "__main__":
    main()
