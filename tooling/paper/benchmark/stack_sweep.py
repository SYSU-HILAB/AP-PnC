"""Stack-level sweep for the SE3 aerodynamic feedforward.

Drives ``infra/sim_infra/simple_sim/stack/stack.sh run`` over a matrix of
aerodynamic models, speed caps, controller variants and SE3 gain scales, and
prints one table at the end. This is deliberately a thin driver: every number in
the table comes from a normal run's manifest/metrics, so the same artefacts can
be inspected by hand afterwards.

Variants
--------
``se3``      SE3 controller, flatness feedforward off (the baseline)
``se3-ff``   SE3 controller, flatness feedforward on
``nmpc``     NMPC baseline for reference

The feedforward variant must be better than ``se3`` *at the same gain scale* for
the comparison to count; the table reports that delta per row.
"""

from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

from tooling.env import artifact_path, project_path, setup_env

VARIANTS = ("se3", "se3-ff", "nmpc")
AERO_MODELS = ("none", "lyu", "phi")

_JOB_PREFIX = "JOB="


@dataclass(frozen=True)
class Row:
    aero: str
    v_max: float
    variant: str
    gain_scale: tuple[float, ...]
    status: str
    steps: int
    position_rmse: float
    position_max: float
    velocity_rmse: float
    run_dir: Path

    def as_dict(self) -> dict:
        data = {
            "aero": self.aero,
            "v_max": self.v_max,
            "variant": self.variant,
            "gain_scale": list(self.gain_scale),
            "status": self.status,
            "steps": self.steps,
            "position_rmse_m": self.position_rmse,
            "position_max_m": self.position_max,
            "velocity_rmse_mps": self.velocity_rmse,
            "run_dir": str(self.run_dir),
        }
        return data


def parse_gain_scales(text: str) -> tuple[tuple[float, ...], ...]:
    """Parse ``--gain-scale``: entries separated by ``;``, each ``S`` or ``p,v,a``.

    ``4`` scales every SE3 gain by 4; ``4,4,16`` scales position, velocity and
    attitude separately. The feedforward moves the attitude reference, so the
    attitude bandwidth is the term most likely to need its own value.
    """
    entries: list[tuple[float, ...]] = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        values = tuple(float(v) for v in chunk.split(",") if v.strip())
        if len(values) == 1:
            entries.append((values[0],) * 3)
        elif len(values) == 3:
            entries.append(values)
        else:
            raise ValueError(f"gain scale {chunk!r} must be S or p,v,a")
    return tuple(entries)


def _patch_config(text: str, variant: str, gain_scale: tuple[float, ...]) -> str:
    """Return the simple_sim.yaml text for one variant at one gain scale."""
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant {variant!r}, expected one of {VARIANTS}")

    # Controller selection + feedforward flag.
    controller = "nmpc" if variant == "nmpc" else "se3"
    feedforward = "true" if variant == "se3-ff" else "false"
    text = re.sub(r"^  controller: .*$", f"  controller: {controller}", text, flags=re.M)
    text = re.sub(
        r"^    aero_flatness_feedforward: .*$",
        f"    aero_flatness_feedforward: {feedforward}",
        text,
        flags=re.M,
    )

    # Gains: scale the configured SE3 gains so that both SE3 variants share it.
    for key, scale in zip(
        ("position_gain", "velocity_gain", "attitude_gain"), gain_scale, strict=True
    ):
        match = re.search(rf"^    {key}: \[(.*)\]$", text, flags=re.M)
        if match is None:
            raise ValueError(f"simple_sim.yaml has no se3.{key}")
        values = [float(v) * scale for v in match.group(1).split(",")]
        rendered = ", ".join(f"{v:.6g}" for v in values)
        text = re.sub(
            rf"^    {key}: \[.*\]$",
            f"    {key}: [{rendered}]",
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
    gain_scales: tuple[tuple[float, ...], ...] = ((1.0, 1.0, 1.0),),
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
        for gain_scale in gain_scales:
            for variant in variants:
                config.write_text(_patch_config(original, variant, gain_scale))
                for aero in aeros:
                    for v_max in v_maxs:
                        label = "x".join(f"{v:g}" for v in gain_scale)
                        tag = f"{profile}-{aero}-v{v_max:g}-{variant}-g{label}"
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
                                gain_scale=tuple(gain_scale),
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
    """Markdown table: one row per (gain scale, aero, v_max), one column set per variant."""
    lines: list[str] = []
    for gain_scale in sorted({r.gain_scale for r in rows}):
        selected = [r for r in rows if r.gain_scale == gain_scale]
        present = [v for v in VARIANTS if any(r.variant == v for r in selected)]
        label = "/".join(f"{v:g}" for v in gain_scale)
        lines.append(f"### SE3 gains x position {gain_scale[0]:g} / velocity {gain_scale[1]:g}"
                     f" / attitude {gain_scale[2]:g}\n")
        header = "| aero | v_max | " + " | ".join(f"{v} posRMSE" for v in present)
        header += " | feedforward |"
        lines.append(header)
        lines.append(
            "|---|---:|" + "---:|" * len(present) + "---|"
        )
        for aero in sorted({r.aero for r in selected}):
            for v_max in sorted({r.v_max for r in selected}):
                picked = {r.variant: r for r in selected if r.aero == aero and r.v_max == v_max}
                cells = []
                for variant in present:
                    row = picked.get(variant)
                    if row is None:
                        cells.append("-")
                    else:
                        mark = "" if row.status == "finished" else " (failed)"
                        cells.append(f"{row.position_rmse:.4f}{mark}")
                delta = ""
                base = picked.get("se3")
                ff = picked.get("se3-ff")
                if base is not None and ff is not None and base.position_rmse > 0.0:
                    change = (base.position_rmse - ff.position_rmse) / base.position_rmse * 100.0
                    delta = f"{change:+.1f}%"
                    if ff.status != "finished":
                        delta += " (failed)"
                lines.append(
                    f"| {aero} | {v_max:g} | "
                    + " | ".join(cells)
                    + f" | {delta} |"
                )
        lines.append("")
    return "\n".join(lines)
