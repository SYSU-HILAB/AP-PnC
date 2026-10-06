"""Stack-level sweep for the SE3 aerodynamic feedforward.

Drives ``infra/sim_infra/simple_sim/stack/stack.sh run`` over a matrix of
aerodynamic models, speed caps and controller variants, and prints one table at
the end. This is deliberately a thin driver: every number in the table comes from
a normal run's manifest/metrics, so the same artefacts can be inspected by hand
afterwards. Controller gains are not a dimension here — the tuned values live in
``core/bringup/config/simple_sim.yaml``, which is the single source of truth for
a run's configuration.

Variants
--------
``se3``      SE3 controller with the configured feedforward setting
``se3-base`` SE3 controller with the aerodynamic feedforward forced off
``nmpc``     NMPC baseline for reference

The table reports the change from ``se3-base`` to ``se3`` on every row, so the
feedforward has to be better than its own baseline on all of them.
"""

from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

from tooling.env import artifact_path, project_path, setup_env

VARIANTS = ("se3", "se3-base", "nmpc")
AERO_MODELS = ("none", "lyu", "phi")

_JOB_PREFIX = "JOB="


@dataclass(frozen=True)
class Row:
    aero: str
    v_max: float
    variant: str
    status: str
    steps: int
    position_rmse: float
    position_max: float
    velocity_rmse: float
    run_dir: Path

    def as_dict(self) -> dict:
        return {
            "aero": self.aero,
            "v_max": self.v_max,
            "variant": self.variant,
            "status": self.status,
            "steps": self.steps,
            "position_rmse_m": self.position_rmse,
            "position_max_m": self.position_max,
            "velocity_rmse_mps": self.velocity_rmse,
            "run_dir": str(self.run_dir),
        }


def _patch_config(text: str, variant: str) -> str:
    """Return the simple_sim.yaml text with the controller of one variant."""
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant {variant!r}, expected one of {VARIANTS}")
    controller = "nmpc" if variant == "nmpc" else "se3"
    text = re.sub(r"^  controller: .*$", f"  controller: {controller}", text, flags=re.M)
    if variant == "se3-base":
        text = re.sub(
            r"^    aero_flatness_feedforward: .*$",
            "    aero_flatness_feedforward: false",
            text,
            flags=re.M,
        )
    return text


def _run_once(
    *,
    aero: str,
    v_max: float,
    duration_s: float,
    profile: str,
    hold_s: float,
    mass_kg: float,
    log_dir: Path,
    tag: str,
) -> tuple[str, Path]:
    stack = project_path("infra/sim_infra/simple_sim/stack/stack.sh")
    log = log_dir / f"{tag}.log"
    command = [
        "bash",
        str(stack),
        "run",
        str(duration_s),
        profile,
        aero,
        str(v_max),
        str(hold_s),
        str(mass_kg),
    ]
    with log.open("w") as handle:
        process = subprocess.run(  # noqa: S603
            command,
            env=setup_env(),
            stdout=handle,
            stderr=subprocess.STDOUT,
            check=False,
        )
    text = log.read_text()
    if process.returncode != 0:
        raise RuntimeError(f"{tag} failed ({process.returncode}); see {log}")
    jobs = [line[len(_JOB_PREFIX) :] for line in text.splitlines() if line.startswith(_JOB_PREFIX)]
    if not jobs:
        raise RuntimeError(f"{tag} produced no JOB= line; see {log}")
    return jobs[-1], log


def run_matrix(
    *,
    aeros: tuple[str, ...] = ("lyu",),
    v_maxs: tuple[float, ...] = (8.0, 10.0, 12.0),
    variants: tuple[str, ...] = VARIANTS,
    duration_s: float = 0.0,
    profile: str = "ideal",
    hold_s: float = 2.0,
    mass_kg: float = 2.0,
    output_dir: Path | None = None,
) -> list[Row]:
    """Run the matrix and return one Row per run."""
    config = project_path("core/bringup/config/simple_sim.yaml")
    original = config.read_text()
    out = output_dir or artifact_path(".artifacts/benchmark/stack-sweep")
    out.mkdir(parents=True, exist_ok=True)
    rows: list[Row] = []
    try:
        for variant in variants:
            config.write_text(_patch_config(original, variant))
            for aero in aeros:
                for v_max in v_maxs:
                    tag = f"{profile}-{aero}-v{v_max:g}-{variant}"
                    job, _ = _run_once(
                        aero=aero,
                        v_max=v_max,
                        duration_s=duration_s,
                        profile=profile,
                        hold_s=hold_s,
                        mass_kg=mass_kg,
                        log_dir=out,
                        tag=tag,
                    )
                    benchmark_dir = Path(job) / "benchmark"
                    run_dir = next(benchmark_dir.glob("simple_sim_*"))
                    manifest = json.loads((run_dir / "manifest.json").read_text())
                    metrics = json.loads((run_dir / "metrics.json").read_text())
                    rows.append(
                        Row(
                            aero=aero,
                            v_max=v_max,
                            variant=variant,
                            status=manifest["status"],
                            steps=int(manifest["committed_steps"]),
                            position_rmse=float(metrics["position_rmse_m"]),
                            position_max=float(metrics["position_max_m"]),
                            velocity_rmse=float(metrics["velocity_rmse_mps"]),
                            run_dir=run_dir,
                        )
                    )
    finally:
        config.write_text(original)

    (out / "sweep.json").write_text(json.dumps([r.as_dict() for r in rows], indent=2))
    (out / "table.md").write_text(render_table(rows))
    return rows


def render_table(rows: list[Row]) -> str:
    """Markdown table: one row per (aero, v_max), one column per variant."""
    present = [v for v in VARIANTS if any(r.variant == v for r in rows)]
    lines = [
        "| aero | v_max | " + " | ".join(f"{v} posRMSE" for v in present) + " | feedforward |",
        "|---|---:|" + "---:|" * len(present) + "---|",
    ]
    for aero in sorted({r.aero for r in rows}):
        for v_max in sorted({r.v_max for r in rows}):
            picked = {r.variant: r for r in rows if r.aero == aero and r.v_max == v_max}
            cells = []
            for variant in present:
                row = picked.get(variant)
                if row is None:
                    cells.append("-")
                    continue
                mark = "" if row.status == "finished" else " (failed)"
                cells.append(f"{row.position_rmse:.4f}{mark}")
            delta = ""
            base = picked.get("se3-base")
            feedforward = picked.get("se3")
            if base is not None and feedforward is not None and base.position_rmse > 0.0:
                change = (
                    (base.position_rmse - feedforward.position_rmse) / base.position_rmse * 100.0
                )
                delta = f"{change:+.1f}%"
                if feedforward.status != "finished":
                    delta += " (failed)"
            lines.append(f"| {aero} | {v_max:g} | " + " | ".join(cells) + f" | {delta} |")
    return "\n".join(lines)
