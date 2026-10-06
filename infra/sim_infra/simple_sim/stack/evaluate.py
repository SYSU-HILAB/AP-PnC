"""Native Docker sweep / offline scientific plots. Never simulate in plot mode."""

import argparse
import csv
import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from tooling.env import artifact_path, get_git_root, project_path

# Only models marked under_test (plus the null control) may appear in a campaign.
# See infra/sim_infra/aerodynamics/include/aerodynamics/aero_models.hpp.
MODELS = ("none", "lyu", "phi")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open() as f:
        data = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in data]) for k in data[0]} if data else {}


def reference_speed(path):
    """Maximum speed at endpoints and all real stationary points, not labels."""
    maximum = 0.0
    for piece in yaml.safe_load(path.read_text())["pieces"]:
        velocity = [np.polyder(v) for v in piece["coefficients_xyz_descending"]]
        squared = np.zeros(1)
        for v in velocity:
            squared = np.polyadd(squared, np.polymul(v, v))
        candidates = [0.0, piece["duration_s"]]
        for r in np.roots(np.polyder(squared)):
            if abs(r.imag) < 1e-7 and 0 < r.real < piece["duration_s"]:
                candidates.append(float(r.real))
        maximum = max(
            maximum, max(float(np.sqrt(max(0, np.polyval(squared, t)))) for t in candidates)
        )
    return maximum


def run(out, profile, duration, vmax=6.0, hold=0.0, mass=None):
    root = get_git_root()
    driver = root / "infra/sim_infra/simple_sim/stack/stack.sh"
    index = {
        "schema": "ap-pnc/tracking-sweep/v1",
        "profile": profile,
        "v_max_setting_mps": vmax,
        "terminal_hold_s": hold,
        "mass_override_kg": mass,
        "runs": [],
    }
    for model in MODELS:
        log = out / f"{model}.log"
        with log.open("w") as f:
            result = subprocess.run(
                [
                    "bash",
                    str(driver),
                    "run",
                    str(duration),
                    profile,
                    model,
                    str(vmax),
                    str(hold),
                    "configured" if mass is None else str(mass),
                ],
                env={**os.environ, "AP_PNC_DIR": str(root)},
                stdout=f,
                stderr=subprocess.STDOUT,
            )
        jobs = [line[4:] for line in log.read_text().splitlines() if line.startswith("JOB=")]
        item = {"model": model, "driver_exit": result.returncode, "log": str(log)}
        if jobs:
            job = project_path(jobs[0])
            manifests = list((job / "benchmark").glob("simple_sim_*/manifest.json"))
            item["job"] = str(job)
            if len(manifests) == 1:
                item["run"] = str(manifests[0].parent)
        index["runs"].append(item)
        (out / "index.json").write_text(json.dumps(index, indent=2))
        print(model, result.returncode, item.get("run", "startup failed"), flush=True)


def plot(out):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    index = json.loads((out / "index.json").read_text())
    vmax = index["v_max_setting_mps"]
    images = out / "figures"
    images.mkdir(exist_ok=True)
    summary = []
    for item in index["runs"]:
        if "run" not in item:
            summary.append({"model": item["model"], "status": "startup unavailable/failed"})
            continue
        folder = project_path(item["run"])
        manifest = json.loads((folder / "manifest.json").read_text())
        integrator = manifest.get("nmpc_solver_abi") == 3
        yb_start = 6 if integrator else 9
        data = rows(folder / "steps.csv")
        trace = (
            rows(folder / "nmpc_reference.csv") if (folder / "nmpc_reference.csv").exists() else {}
        )
        entry = {
            "model": item["model"],
            "status": manifest["status"],
            "stop_time_s": manifest["time_ns"] * 1e-9,
            "steps": manifest["committed_steps"],
            "trajectory_duration_s": manifest["trajectory_duration_s"],
            "terminal_hold_s": manifest.get("terminal_hold_s", 0.0),
            "reference_terminal_policy": manifest.get("reference_terminal_policy", "error"),
            "v_max_setting_mps": vmax,
            "reason": manifest["reason"],
            "reference_sha256": sha(folder / "reference.yaml"),
            "reference_max_speed_mps": reference_speed(folder / "reference.yaml"),
            "source_sha256": {
                p.name: sha(p)
                for p in folder.iterdir()
                if p.suffix in (".csv", ".yaml") or p.name in ("manifest.json", "metrics.json")
            },
            "metrics": json.loads((folder / "metrics.json").read_text()),
            "metrics_scope": "complete with final sample"
            if manifest["status"] == "finished"
            else "partial pre-failure samples; NOT full-run performance",
            "verified_replay": (folder / "verified.json").is_file(),
        }
        summary.append(entry)
        if not data:
            continue
        entry["actual_max_speed_mps"] = float(
            max(
                np.max(np.linalg.norm(np.column_stack([data[f"v_{a}"] for a in "xyz"]), axis=1)),
                np.linalg.norm([data[f"next_v_{a}"][-1] for a in "xyz"]),
            )
        )
        ocp = json.loads((project_path(item["job"]) / "generated-ocp.json").read_text())
        entry["cost_diagonal_stage0"] = np.diag(ocp["cost"]["W_0"]).tolist()
        entry["cost_diagonal_future"] = np.diag(ocp["cost"]["W"]).tolist()
        label = f'{item["model"]} | {manifest["status"].upper()} | {entry["stop_time_s"]:.2f}s | v_max setting={vmax:g}'
        t = data["time_ns"] * 1e-9
        fig = plt.figure(figsize=(8, 6), constrained_layout=True)
        axis = fig.add_subplot(111, projection="3d")
        axis.plot(*(data[f"ref_p_{a}"] for a in "xyz"), label="reference ENU", color="#cc8b19")
        axis.plot(*(data[f"p_{a}"] for a in "xyz"), label="actual ENU", color="#315d93")
        axis.set_xlabel("East [m]")
        axis.set_ylabel("North [m]")
        axis.set_zlabel("Up [m]")
        axis.set_title(label)
        axis.legend()
        fig.savefig(images / f"{item['model']}-path.png", dpi=130)
        plt.close(fig)
        fig, axes = plt.subplots(4, 3, figsize=(14, 12), sharex=True, constrained_layout=True)
        for a, axis in enumerate("xyz"):
            for row, prefix, unit in [(0, "p", "m"), (1, "v", "m/s")]:
                axes[row, a].plot(
                    t, data[f"ref_{prefix}_{axis}"], label="planner reference", color="#cc8b19"
                )
                axes[row, a].plot(t, data[f"{prefix}_{axis}"], label="actual", color="#315d93")
                axes[row, a].set_title(f"{prefix}_{axis} [{unit}] (ENU)")
            axes[2, a].plot(t, data[f"rate_{axis}_sp"], label="optimized u0 rate_sp")
            axes[2, a].plot(t, data[f"w_{axis}"], label="actual omega_B")
            axes[2, a].axhline(
                0, color="#cc8b19", ls="--", label="actual solver rate reference = 0"
            )
            axes[2, a].set_title(f"omega_{axis} [rad/s] (FLU)")
        q = np.column_stack([data[f"q_{a}"] for a in "wxyz"])
        qw, qx, qy, qz = q.T
        y_actual = np.column_stack(
            [2 * (qx * qy - qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + qw * qx)]
        )
        y_ref = np.column_stack([data[f"ref_yb_{a}"] for a in "xyz"])
        for a, axis in enumerate("xyz"):
            axes[3, a].plot(t, y_ref[:, a], label="planner yb_W")
            axes[3, a].plot(t, y_actual[:, a], label="actual body Y in world")
            axes[3, a].set_title(f"yb_{axis} (ENU direction)")
        for ax in axes.flat:
            ax.grid(alpha=0.2)
            ax.legend(fontsize=7)
            ax.set_xlabel("simulation time [s]")
        fig.suptitle(label + " | STOP = partial diagnostic if FAILED", fontsize=12)
        fig.savefig(images / f"{item['model']}-tracking.png", dpi=130)
        plt.close(fig)
        fig, ax = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
        error = np.linalg.norm(
            np.column_stack([data[f"p_{a}"] - data[f"ref_p_{a}"] for a in "xyz"]), axis=1
        )
        angle = np.degrees(np.arccos(np.clip(np.sum(y_actual * y_ref, axis=1), -1, 1)))
        ax[0, 0].plot(t, error)
        ax[0, 0].set_title("Position error [m]")
        ax[0, 1].plot(t, angle)
        ax[0, 1].set_title("Body-Y direction error [deg]")
        ax[1, 0].plot(t, data["specific_force_sp"], label="published collective command")
        mass = yaml.safe_load((folder / "simple_sim.yaml").read_text())["simple_sim"]["plant"][
            "mass"
        ]
        ax[1, 0].plot(t, data["applied_fz"] / mass, label="actuator Fz/m")
        ax[1, 0].plot(t, data["imu_z"], label="net IMU z (includes aero)")
        ax[1, 0].axhline(
            9.81,
            ls="--",
            label=(
                "hover scale, NOT a cost target" if integrator else "solver command reference hover"
            ),
        )
        ax[1, 0].set_title("Collective / body-Z specific force [N/kg]")
        ax[1, 0].legend(fontsize=7)
        for a in "xyz":
            ax[1, 1].plot(t, data[f"aero_m{a}"], label=f"aero M{a}")
        ax[1, 1].set_title("Aerodynamic torque [Nm], FLU")
        ax[1, 1].legend()
        for a in ax.flat:
            a.grid(alpha=0.2)
            a.set_xlabel("simulation time [s]")
        fig.suptitle(label)
        fig.savefig(images / f"{item['model']}-diagnostics.png", dpi=130)
        plt.close(fig)
        if trace:
            mask = trace["stage"] == 0
            tc = trace["control_time_ns"][mask] * 1e-9
            fig, axes = plt.subplots(4, 3, figsize=(14, 11), constrained_layout=True)
            for a, axis in enumerate("xyz"):
                for row, start, planner in [(0, 0, "p"), (1, 3, "v"), (2, yb_start, "yb")]:
                    axes[row, a].plot(
                        tc, trace[f"planner_{planner}_{axis}"][mask], label="planner", lw=2
                    )
                    axes[row, a].plot(
                        tc, trace[f"yref_{start+a}"][mask], label="ACTUAL SOLVER YREF", ls="--"
                    )
                    axes[row, a].set_title(f"{planner}_{axis} (world ENU)")
                axes[3, a].plot(
                    tc, trace[f"planner_omega_{axis}"][mask], label="new physical FLU planned omega"
                )
                if integrator:
                    axes[3, a].plot(
                        tc,
                        trace[f"initial_state_{6+a}"][mask],
                        label="initial rate COMMAND (not a cost target)",
                    )
                    axes[3, a].plot(
                        tc,
                        trace[f"published_command_{1+a}"][mask],
                        label="integrated rate command",
                        ls="--",
                    )
                else:
                    axes[3, a].plot(
                        tc, trace[f"yref_{6+a}"][mask], label="solver STATE-rate reference"
                    )
                    axes[3, a].plot(
                        tc,
                        trace[f"yref_{13+a}"][mask],
                        label="solver COMMAND-rate reference",
                        ls="--",
                    )
                axes[3, a].set_title(f"rate_{axis} (body FLU)")
            for a in axes.flat:
                a.grid(alpha=0.2)
                a.legend(fontsize=7)
                a.set_xlabel("control time [s]")
            fig.suptitle(
                label
                + (
                    " | ABI3: p/v/yb + command derivatives; W0 active"
                    if integrator
                    else " | historical ABI2: W0=0; future knots weighted"
                ),
                fontsize=11,
            )
            fig.savefig(images / f"{item['model']}-nmpc-reference.png", dpi=130)
            plt.close(fig)
            entry["planner_solver_yb_max_difference"] = max(
                float(np.max(np.abs(trace[f"planner_yb_{a}"] - trace[f"yref_{yb_start+i}"])))
                for i, a in enumerate("xyz")
            )
            yr = np.column_stack([trace[f"yref_{yb_start+a}"][mask] for a in range(3)])
            dots = np.sum(yr[1:] * yr[:-1], axis=1)
            flips = np.where(dots < 0)[0]
            entry["yb_negative_adjacent_dot_events"] = [
                {
                    "time_s": float(tc[i + 1]),
                    "dot": float(dots[i]),
                    "fallback_before": bool(trace["fallback"][mask][i]),
                    "fallback_after": bool(trace["fallback"][mask][i + 1]),
                }
                for i in flips
            ]
            entry["yb_min_adjacent_dot"] = float(np.min(dots)) if len(dots) else None
            entry["solver_failed_ticks"] = list(
                map(int, trace["tick"][mask][trace["solver_status"][mask] == 4])
            )
            fig, axes = plt.subplots(2, 3, figsize=(12, 7), constrained_layout=True)
            # Plot future knots at THEIR reference time, never at current-state time.
            for tick in (100, 300, 600):
                selected = trace["tick"] == tick
                if not np.any(selected):
                    continue
                future_time = trace["reference_time_ns"][selected] * 1e-9
                for a in range(3):
                    axes[0, a].plot(
                        future_time,
                        trace[f"yref_{a}"][selected],
                        "o-",
                        label=f"horizon at t={tick*.02:.1f}s",
                    )
                    axes[1, a].plot(future_time, trace[f"yref_{yb_start+a}"][selected], "o-")
            for a, axis in enumerate("xyz"):
                axes[0, a].set_title(f"p_{axis} solver future knots [m]")
                axes[1, a].set_title(f"yb_{axis} solver future knots")
            for a in axes.flat:
                a.grid(alpha=0.2)
                a.set_xlabel("REFERENCE time [s]")
            axes[0, 0].legend(fontsize=8)
            fig.suptitle(
                "Logged solver horizons: stage10 terminal; "
                + ("stage0 derivative cost active" if integrator else "historical stage0 W=0")
            )
            fig.savefig(images / f"{item['model']}-horizons.png", dpi=130)
            plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for item in index["runs"]:
        if "run" not in item:
            continue
        data = rows(project_path(item["run"]) / "steps.csv")
        if not data:
            continue
        t = data["time_ns"] * 1e-9
        e = np.linalg.norm(
            np.column_stack([data[f"p_{a}"] - data[f"ref_p_{a}"] for a in "xyz"]), axis=1
        )
        speed = np.linalg.norm(np.column_stack([data[f"v_{a}"] for a in "xyz"]), axis=1)
        axes[0].plot(t, e, label=item["model"])
        axes[1].plot(t, speed, label=item["model"])
    axes[0].set_title("Matched-window position errors [m]")
    axes[1].axhline(vmax, color="gray", ls="--", label=f"v_max setting={vmax:g} (soft constraint)")
    axes[1].set_title("Actual speed [m/s]")
    for a in axes:
        a.grid(alpha=0.2)
        a.legend()
        a.set_xlabel("simulation time [s]")
    fig.savefig(images / "comparison.png", dpi=150)
    plt.close(fig)
    same = (
        len(summary) == len(index["runs"])
        and len({r["reference_sha256"] for r in summary if "reference_sha256" in r}) == 1
    )
    report = {
        "schema": "ap-pnc/tracking-evaluation/v1",
        "same_saved_reference": same,
        "v_max_setting_mps": vmax,
        "terminal_hold_s": index.get("terminal_hold_s", 0.0),
        "models": summary,
        "frame_scope": "public reference now direct FLU; legacy MINCO cost kernel still pending migration",
        "reference_semantics": "ABI3: p/v/yb + zero command derivatives, W0 active; legacy ABI2 retains its original layout and weights",
        "plots": "raw logged data only; failed plots are partial diagnostics, no successful replay marker",
    }
    (out / "summary.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["run", "plot"])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--profile", choices=["ideal", "practical"], default="practical")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--v-max", type=float, default=6.0)
    parser.add_argument("--hold-seconds", type=float, default=0.0)
    parser.add_argument(
        "--mass", type=float, help="explicit plant/planner/controller mass override [kg]"
    )
    args = parser.parse_args()
    if not np.isfinite(args.v_max) or args.v_max <= 0:
        parser.error("positive finite v-max required")
    if not np.isfinite(args.hold_seconds) or args.hold_seconds < 0:
        parser.error("nonnegative finite hold-seconds required")
    if args.mass is not None and (not np.isfinite(args.mass) or args.mass <= 0):
        parser.error("positive finite mass required")
    if args.output and not args.output.is_absolute():
        parser.error("absolute output required")
    out = artifact_path(
        args.output
        or (
            get_git_root()
            / ".artifacts/benchmark"
            / (f"tracking-v{args.v_max:g}-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
        )
    )
    out.mkdir(parents=True, exist_ok=True)
    if args.mode == "run":
        if (out / "index.json").exists():
            parser.error("refusing to overwrite existing sweep")
        run(out, args.profile, args.duration, args.v_max, args.hold_seconds, args.mass)
    else:
        plot(out)
    print("EVALUATION=" + str(out))


if __name__ == "__main__":
    main()
