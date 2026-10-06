"""Stack-level sweep for the SE3 aerodynamic feedforward.

Runs a matrix of aerodynamic models, speed caps, actuator profiles and controller
variants through :mod:`tooling.simple_stack` — which owns the whole job lifecycle
— and prints one table at the end. This is deliberately a thin driver: every number in the table comes from
a normal run's manifest/metrics, so the same artefacts can be inspected by hand
afterwards. Controller gains are not a dimension here — the tuned values live in
``core/bringup/config/simple_sim.yaml``, which is the single source of truth for
a run's configuration.

Variants
--------
``se3-aero-ff`` SE3 controller with the aerodynamic feedforward forced ON
``se3-base``    the same controller with the feedforward forced OFF
``nmpc``        NMPC baseline for reference

Both SE3 variants force their setting, so the table means the same thing whatever
the configured default happens to be.

The table reports the change from ``se3-base`` to ``se3`` on every row, so the
feedforward has to be better than its own baseline on all of them.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

from tooling import simple_stack
from tooling.env import artifact_path

VARIANTS = ("se3-aero-ff", "se3-base", "nmpc")
AERO_MODELS = ("none", "lyu", "phi")

_JOB_PREFIX = "JOB="


@dataclass(frozen=True)
class Row:
    profile: str
    aero: str
    v_max: float
    variant: str
    status: str
    steps: int
    position_rmse: float
    position_max: float
    velocity_rmse: float
    max_sideslip_deg: float
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
            "max_sideslip_deg": self.max_sideslip_deg,
            "run_dir": str(self.run_dir),
        }


def _patch_config(text: str, variant: str) -> str:
    """Return the simple_sim.yaml text with the controller of one variant."""
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant {variant!r}, expected one of {VARIANTS}")
    controller = "nmpc" if variant == "nmpc" else "se3"
    text = re.sub(r"^  controller: .*$", f"  controller: {controller}", text, flags=re.M)
    if variant != "nmpc":
        enabled = "true" if variant == "se3-aero-ff" else "false"
        text = re.sub(
            r"^    aero_flatness_feedforward: .*$",
            f"    aero_flatness_feedforward: {enabled}",
            text,
            flags=re.M,
        )
    return text


def _variant_overrides(variant: str) -> dict:
    """What a variant means, as overrides for the job's own configuration copy.

    A variant is a run parameter, not an edit to a tracked file, so a sweep never
    rewrites ``core/bringup/config`` and two sweeps cannot race on it.
    """
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant {variant!r}, expected one of {VARIANTS}")
    overrides: dict[str, object] = {
        "simple_sim.controller": "nmpc" if variant == "nmpc" else "se3"
    }
    if variant != "nmpc":
        overrides["simple_sim.se3.aero_flatness_feedforward"] = variant == "se3-aero-ff"
    return overrides


def _run_once(
    *,
    aero: str,
    v_max: float,
    duration_s: float,
    profile: str,
    hold_s: float,
    mass_kg: float,
    overrides: dict,
    log_dir: Path,
    tag: str,
) -> Path:
    """One job, driven through the tooling.

    The lifecycle (job directory, configuration overrides, containerised dora run,
    receipt check) is :mod:`tooling.simple_stack`; the shell keeps only the build.
    """
    job = simple_stack.run_job(
        simple_stack.JobRequest(
            profile=profile,
            aero=aero,
            v_max=v_max,
            hold_s=hold_s,
            mass_kg=mass_kg,
            duration_s=duration_s,
            overrides=overrides,
        )
    )
    (log_dir / f"{tag}.log").write_text(str(job / "stack.log"))
    (log_dir / f"{tag}.job").write_text(str(job))
    return job


def run_matrix(
    *,
    aeros: tuple[str, ...] = ("lyu",),
    v_maxs: tuple[float, ...] = (8.0, 10.0, 12.0),
    variants: tuple[str, ...] = VARIANTS,
    duration_s: float = 0.0,
    profiles: tuple[str, ...] = ("practical",),
    hold_s: float = 2.0,
    mass_kg: float = 2.0,
    output_dir: Path | None = None,
) -> list[Row]:
    """Run the matrix and return one Row per run."""
    out = output_dir or artifact_path(".artifacts/benchmark/stack-sweep")
    out.mkdir(parents=True, exist_ok=True)
    rows: list[Row] = []
    for variant in variants:
        overrides = _variant_overrides(variant)
        for profile in profiles:
            for aero in aeros:
                for v_max in v_maxs:
                    tag = f"{profile}-{aero}-v{v_max:g}-{variant}"
                    # One bad run must not destroy the table: a failure is a row,
                    # not an exception. A real crash of the stack still raises out
                    # of run_job and is reported as an error row.
                    try:
                        job = _run_once(
                            aero=aero,
                            v_max=v_max,
                            duration_s=duration_s,
                            profile=profile,
                            hold_s=hold_s,
                            mass_kg=mass_kg,
                            overrides=overrides,
                            log_dir=out,
                            tag=tag,
                        )
                        summary = simple_stack.run_summary(job)
                        run_dir = summary["run_dir"]
                        manifest = summary["manifest"]
                        metrics = summary["metrics"]
                        row = Row(
                            profile=profile,
                            aero=aero,
                            v_max=v_max,
                            variant=variant,
                            status=manifest["status"],
                            steps=int(manifest["committed_steps"]),
                            position_rmse=float(metrics["position_rmse_m"]),
                            position_max=float(metrics["position_max_m"]),
                            velocity_rmse=float(metrics["velocity_rmse_mps"]),
                            max_sideslip_deg=float(
                                metrics.get("max_sideslip_deg", float("nan"))
                            ),
                            run_dir=run_dir,
                        )
                    except Exception as error:  # - keep sweeping
                        print(f"{tag}: {error}")
                        row = Row(
                            profile=profile,
                            aero=aero,
                            v_max=v_max,
                            variant=variant,
                            status="error",
                            steps=0,
                            position_rmse=float("nan"),
                            position_max=float("nan"),
                            velocity_rmse=float("nan"),
                            run_dir=out,
                        )
                    rows.append(row)
                    print(
                        f"{profile:9s} {aero:5s} v{v_max:<3g} {variant:12s} "
                        f"{row.status:9s} {row.position_rmse:.4f}",
                        flush=True,
                    )

    (out / "sweep.json").write_text(json.dumps([r.as_dict() for r in rows], indent=2))
    (out / "table.md").write_text(render_table(rows))
    return rows


def _cell(row: Row | None) -> str:
    """One table cell: a number with an optional failure mark, or an error."""
    if row is None:
        return "-"
    if row.status == "error" or row.position_rmse != row.position_rmse:  # NaN
        return "error"
    mark = "" if row.status == "finished" else " (failed)"
    beta = row.max_sideslip_deg
    sideslip = f" / {beta:.1f}" if beta == beta else ""  # NaN-tolerant
    return f"{row.position_rmse:.4f}{sideslip}{mark}"


def render_table(rows: list[Row]) -> str:
    """Markdown tables: one per profile, one row per (aero, v_max)."""
    present = [v for v in VARIANTS if any(r.variant == v for r in rows)]
    lines: list[str] = []
    for profile in sorted({r.profile for r in rows}):
        selected = [r for r in rows if r.profile == profile]
        lines.append(f"### actuator profile: {profile}\n")
        lines.append(
            "| aero | v_max | " + " | ".join(f"{v} posRMSE" for v in present)
            + " | feedforward |"
        )
        lines.append("|---|---:|" + "---:|" * len(present) + "---|")
        for aero in sorted({r.aero for r in selected}):
            for v_max in sorted({r.v_max for r in selected}):
                picked = {
                    r.variant: r
                    for r in selected
                    if r.aero == aero and r.v_max == v_max
                }
                cells = [_cell(picked.get(v)) for v in present]
                delta = ""
                base = picked.get("se3-base")
                feedforward = picked.get("se3-aero-ff")
                if (
                    base is not None
                    and feedforward is not None
                    and base.position_rmse > 0.0
                    and feedforward.position_rmse == feedforward.position_rmse
                ):
                    change = (
                        (base.position_rmse - feedforward.position_rmse)
                        / base.position_rmse
                        * 100.0
                    )
                    delta = f"{change:+.1f}%"
                    if feedforward.status != "finished":
                        delta += " (failed)"
                lines.append(
                    f"| {aero} | {v_max:g} | " + " | ".join(cells) + f" | {delta} |"
                )
        lines.append("")
    return "\n".join(lines)
