"""Plot recorded model angles/FLU aero forces, never rerun the simulation."""

import argparse
import csv
import json

import matplotlib
import numpy as np

from tooling.env import artifact_path, project_path

matplotlib.use("Agg")


def committed_series(folder):
    with (folder / "steps.csv").open() as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError("no committed states for aerodynamic history")
    time = np.array([float(r["time_ns"]) for r in rows] + [float(rows[-1]["next_time_ns"])]) * 1e-9

    def series(name):
        return np.array([float(r[name]) for r in rows] + [float(rows[-1]["next_" + name])])

    valid = series("aero_observation_valid")
    if not np.all(valid == 1):
        raise ValueError("actual aerodynamic model observations unavailable")
    velocity = np.column_stack([series(f"air_wing_v_{a}") for a in "xyz"])
    forces = np.column_stack([series(f"aero_f{a}") for a in "xyz"])
    moments = np.column_stack([series(f"aero_m{a}") for a in "xyz"])
    angles = np.column_stack([series("aero_alpha_rad"), series("aero_beta_rad")])
    if not all(np.isfinite(x).all() for x in (time, velocity, forces, moments, angles)):
        raise ValueError("nonfinite aerodynamic diagnostic")
    return time, angles, velocity, forces, moments


def plot(out):
    import matplotlib.pyplot as plt

    out = artifact_path(out)
    index = json.loads((out / "index.json").read_text())
    images = out / "figures"
    images.mkdir(exist_ok=True)
    summaries = []
    for item in index["runs"]:
        if item["model"] not in {"none", "lyu", "ma"}:
            raise ValueError("this campaign explicitly excludes phi/advanced")
        folder = project_path(item["run"])
        manifest = json.loads((folder / "manifest.json").read_text())
        time, angles, velocity, forces, moments = committed_series(folder)
        speed = np.linalg.norm(velocity, axis=1)
        low_speed = speed < 0.5
        fig, axes = plt.subplots(3, 2, figsize=(12, 11), constrained_layout=True)
        for i, name in enumerate(("Angle of attack alpha [deg]", "Sideslip beta [deg]")):
            ax = axes[0, i]
            ax.plot(time, np.degrees(angles[:, i]), lw=1.0, label="ACTUAL model-returned angle")
            ax.fill_between(
                time,
                0,
                1,
                where=low_speed,
                transform=ax.get_xaxis_transform(),
                color="grey",
                alpha=0.15,
                label="air speed < 0.5 m/s",
            )
            ax.set_title(name + " (wing FRD)")
        for i, a in enumerate("xyz"):
            ax = axes.flat[i + 2]
            ax.plot(time, forces[:, i], label=f"aerodynamic F{a}, body FLU", color=f"C{i}")
            ax.set_title(f"Aerodynamic F{a} [N] (no gravity/rotor thrust)")
        axes[2, 1].plot(time, speed, label="air-relative speed [m/s]")
        axes[2, 1].axhline(
            index["v_max_setting_mps"], ls="--", color="grey", label="planner soft v_max setting"
        )
        axes[2, 1].set_title("Actual air speed [m/s]")
        for ax in axes.flat:
            ax.grid(alpha=0.2)
            ax.set_xlabel("simulation time [s], including final committed state")
            ax.legend(fontsize=8)
        target = (
            np.ceil(
                (manifest["trajectory_duration_s"] + manifest["terminal_hold_s"])
                / (manifest["control_dt_ns"] * 1e-9)
            )
            * manifest["control_dt_ns"]
            * 1e-9
        )
        status = "COMPLETE" if manifest["status"] == "finished" else "FAILED / PARTIAL"
        fig.suptitle(
            f"ABI3 | 2.0 kg FOUR-MOTOR | v_max={index['v_max_setting_mps']:g} m/s | "
            f"{item['model']} | {status}\n"
            f"committed {time[-1]:.2f}s / requested {target:.2f}s; "
            "wing velocity=(body Z,-body Y,body X); raw wrapped angles",
            fontsize=12,
        )
        path = images / f"{item['model']}-aerodynamic-history.png"
        fig.savefig(path, dpi=150)
        plt.close(fig)
        summaries.append(
            {
                "model": item["model"],
                "status": manifest["status"],
                "committed_end_s": float(time[-1]),
                "requested_end_s": float(target),
                "angle_semantics": "actual model-returned wing-FRD radians; low-speed guards model-owned",
                "force_semantics": "aerodynamic only, body FLU, N; final committed state included",
                "max_abs_alpha_deg": float(np.max(np.abs(np.degrees(angles[:, 0])))),
                "max_abs_beta_deg": float(np.max(np.abs(np.degrees(angles[:, 1])))),
                "max_abs_force_xyz_N": np.max(np.abs(forces), axis=0).tolist(),
                "max_abs_moment_xyz_Nm": np.max(np.abs(moments), axis=0).tolist(),
                "figure": str(path),
            }
        )
    (out / "aerodynamic-summary.json").write_text(json.dumps(summaries, indent=2))
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(json.dumps(plot(project_path(args.output)), indent=2))


if __name__ == "__main__":
    main()
