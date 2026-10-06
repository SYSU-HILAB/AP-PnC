"""Run the offline simple_sim stack from Python.

The whole job lifecycle lives here: the job directory and its metadata, the
configuration overrides, the containerised ``dora run`` and the receipt check.
``stack.sh`` keeps only the build and the optional services, and anything that
wants a run — ``ap-pnc benchmark``, tests, a notebook — calls this module instead
of a shell script. One Python process can therefore own a whole sweep and print
one table at the end.

Layout of a job (all under ``.artifacts/simple-stack/<os>-<arch>/``):

    runs/<utc>-<pid>/config/    the exact yaml the run used
    runs/<utc>-<pid>/benchmark/ the simulator output and the verification receipt
    runs/<utc>-<pid>/stack.log  the graph's stdout
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import shutil
import subprocess
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Final

import yaml

from tooling.env import artifact_path, get_git_root, project_path, setup_env

#: Canonical aerodynamic models the driver accepts; the validation status of each
#: lives in aerodynamics/aero_models.hpp and "configured" leaves the yaml alone.
AERO_MODELS: Final = ("configured", "none", "lyu", "phi")
PROFILES: Final = ("ideal", "practical")

_STACK_DIR: Final = "infra/sim_infra/simple_sim/stack"
_JOB_IN_CONTAINER: Final = "/workspace/.artifacts/simple-stack/job"


def native_arch() -> str:
    """Docker platform suffix for this host, normalised like the CLI does."""
    machine = platform.machine().lower()
    if machine in {"arm64", "aarch64"}:
        return "arm64"
    if machine in {"x86_64", "amd64"}:
        return "amd64"
    raise RuntimeError(f"unsupported architecture: {machine}")


def solver_arch() -> str:
    return "aarch64" if native_arch() == "arm64" else "x86_64"


def stack_root() -> Path:
    """Per-architecture stack directory under .artifacts."""
    return artifact_path(f".artifacts/simple-stack/linux-{native_arch()}")


def stack_image() -> str:
    return f"ap-pnc-simple-stack:{native_arch()}"


@dataclass(frozen=True)
class JobRequest:
    """One run of the stack: the same knobs ``stack.sh run`` took positionally."""

    profile: str = "ideal"
    aero: str = "configured"
    v_max: float | None = None
    hold_s: float = 0.0
    mass_kg: float | None = None
    duration_s: float = 0.0

    #: Dotted yaml overrides applied to the job's own configuration copy, keyed by
    #: the document root ("simple_sim.controller", "problem_formulation.cost.v_max",
    #: ...). A variant is therefore a run parameter, not an edit to a tracked file.
    overrides: Mapping[str, object] = field(default_factory=dict)

    def validate(self) -> None:
        if self.profile not in PROFILES:
            raise ValueError(f"profile must be one of {PROFILES}, got {self.profile!r}")
        if self.aero not in AERO_MODELS:
            raise ValueError(f"aero must be one of {AERO_MODELS}, got {self.aero!r}")
        for name, value in (
            ("duration_s", self.duration_s),
            ("hold_s", self.hold_s),
        ):
            if value is None or not (value >= 0.0):
                raise ValueError(f"{name} must be finite and non-negative")
        for name, value in (("v_max", self.v_max), ("mass_kg", self.mass_kg)):
            if value is not None and not value > 0.0:
                raise ValueError(f"{name} must be positive when given")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return f"{digest.hexdigest()}  {path}\n"


def _container(job: Path, *args: str, with_simulator: bool = True) -> subprocess.CompletedProcess:
    """Run a command inside the stack runtime image with the job mounted.

    Faithful to the mounts the shell used: the image keeps its own compiled C++
    install, the job owns the simulator output and the configuration, and nothing
    else of the host tree is exposed.
    """
    root = stack_root()
    command = [
        "docker",
        "run",
        "--rm",
        "--platform",
        f"linux/{native_arch()}",
        "--ipc=host",
        "--network",
        "none",
        "-e",
        "AP_PNC_DIR=/workspace",
        "-e",
        "RERUN_ANALYTICS_ENABLED=false",
        "-e",
        f"HOME={_JOB_IN_CONTAINER}",
        "--workdir",
        _JOB_IN_CONTAINER,
        "-v",
        f"{root}/target/release/ap-pnc-offline:/workspace/.artifacts/bin/ap-pnc-offline:ro",
    ]
    if with_simulator:
        command += [
            "-v",
            f"{root}/cpp/install/lib/simple_sim/simple_sim_run:"
            "/workspace/.artifacts/colcon/install/lib/simple_sim/simple_sim_run:ro",
        ]
    command += [
        "-v",
        f"{job}:{_JOB_IN_CONTAINER}:rw",
        "-v",
        f"{job}/benchmark:/workspace/.artifacts/benchmark:rw",
        "-v",
        f"{job}/config:/workspace/core/bringup/config:ro",
        stack_image(),
        *args,
    ]
    return subprocess.run(
        command, env=setup_env(), capture_output=True, text=True, check=False
    )


def _write_configuration(job: Path, request: JobRequest) -> None:
    """Apply the profile and the request to the job's own configuration copy.

    Pure yaml work, so it happens here on the host rather than inside a
    throwaway container: the result is the same and the transformation is
    reviewable.
    """
    config_dir = job / "config"
    simple_sim = yaml.safe_load((config_dir / "simple_sim.yaml").read_text())
    simple_sim["simple_sim"]["duration_s"] = request.duration_s
    simple_sim["simple_sim"]["terminal_hold_s"] = request.hold_s

    planning_path = config_dir / "planning.yaml"
    planning = yaml.safe_load(planning_path.read_text())

    if request.profile == "practical":
        profile_path = job / "profile.yaml"
        if not profile_path.exists():
            raise FileNotFoundError(
                f"the practical profile needs {profile_path}; run "
                "'stack.sh import-matlab' first"
            )
        profile = yaml.safe_load(profile_path.read_text())
        simple_sim["simple_sim"]["plant"].update(profile["plant"])
        simple_sim["simple_sim"]["actuator"] = profile["actuator"]
        # Only the planner owns a mass: the NMPC model is mass-normalized.
        planning["problem_formulation"]["flatness"]["mass"] = profile["plant"]["mass"]

    if request.mass_kg is not None:
        original = simple_sim["simple_sim"]["plant"]["mass"]
        simple_sim["simple_sim"]["plant"]["mass"] = request.mass_kg
        planning["problem_formulation"]["flatness"]["mass"] = request.mass_kg
        (job / "experiment-overrides.json").write_text(
            json.dumps(
                {
                    "mass_kg": request.mass_kg,
                    "original_profile_mass_kg": original,
                    "unchanged": (
                        "inertia, motor calibration, PID, weights, bounds and timing"
                    ),
                },
                indent=2,
            )
        )

    if request.aero != "configured":
        simple_sim["simple_sim"]["aero"]["model"] = request.aero
    if request.v_max is not None:
        planning["problem_formulation"]["cost"]["v_max"] = request.v_max

    # Override paths are file-relative and start at the document root the file
    # already carries, e.g. "simple_sim.se3.aero_flatness_feedforward" or
    # "problem_formulation.cost.v_max". Unknown roots are rejected instead of
    # silently writing a key nobody reads.
    documents: dict[str, dict] = {
        "simple_sim": simple_sim,
        "problem_formulation": planning,
    }
    for path, value in request.overrides.items():
        keys = path.split(".")
        root = keys[0]
        if root not in documents or len(keys) < 2:
            raise ValueError(
                f"override {path!r} must start with one of {sorted(documents)}"
            )
        node = documents[root][root]
        for key in keys[1:-1]:
            node = node[key]
        node[keys[-1]] = value

    (config_dir / "simple_sim.yaml").write_text(yaml.safe_dump(simple_sim, sort_keys=False))
    planning_path.write_text(yaml.safe_dump(planning, sort_keys=False))


def _write_metadata(job: Path) -> None:
    """Record what produced the run: binaries, image, solver, source revision."""
    root = stack_root()
    source_dir = project_path(_STACK_DIR)
    offline = root / "target/release/ap-pnc-offline"
    simulator = root / "cpp/install/lib/simple_sim/simple_sim_run"
    if not offline.exists() or not simulator.exists():
        raise FileNotFoundError("run 'stack.sh build' first")

    shutil.copy2(source_dir / "dataflow.yml", job / "dataflow.yml")
    shutil.copy2(source_dir / "Cargo.lock", job / "Cargo.lock")
    inspect = subprocess.run(
        ["docker", "image", "inspect", stack_image(), f"simple-sim:{native_arch()}"],
        env=setup_env(),
        capture_output=True,
        text=True,
        check=False,
    )
    (job / "images.jsonl").write_text(inspect.stdout)
    (job / "generated-ocp.json").write_text(
        (
            artifact_path(
                f".artifacts/nmpc_build/{solver_arch()}/c_generated_code/"
                "acados_ocp_tailsitter_flu.json"
            )
        ).read_text()
    )
    git = get_git_root()
    revision = subprocess.run(
        ["git", "-C", str(git), "rev-parse", "HEAD"], capture_output=True, text=True, check=True
    ).stdout
    status = subprocess.run(
        ["git", "-C", str(git), "status", "--porcelain"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    (job / "source-revision.txt").write_text(revision)
    (job / "source-status.txt").write_text(status)
    (job / "node-binary.sha256").write_text(_sha256(offline))
    (job / "simulator-binary.sha256").write_text(_sha256(simulator))


RUST_STEPS = """
set -euo pipefail
manifest="$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/Cargo.toml"
target="$AP_PNC_DIR/.artifacts/simple-stack/cache/target"
if [[ ! -f "${manifest%/*}/Cargo.lock" ]]; then cargo generate-lockfile --manifest-path "$manifest"; fi
cargo fmt --manifest-path "$manifest" -- --check
cargo test --locked --release --jobs 2 --manifest-path "$manifest" --target-dir "$target"
cargo build --locked --release --jobs 2 --manifest-path "$manifest" --target-dir "$target"
cargo clippy --locked --release --jobs 2 --manifest-path "$manifest" --target-dir "$target" -- -D warnings
"""

CPP_STEPS = """
set -euo pipefail
dir="$AP_PNC_DIR/.artifacts/simple-stack/cache/cpp"
solver_arch="$(uname -m)"
cmake -S "$AP_PNC_DIR/infra/sim_infra/simple_sim" -B "$dir/build" \\
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dir/install" \\
  -DSIMPLE_SIM_WITH_ROS=OFF -DSIMPLE_SIM_WITH_NMPC=ON -DBUILD_TESTING=ON \\
  -DNMPC_BUNDLE_PATH="$AP_PNC_DIR/.artifacts/nmpc_solver/libnmpc_bundle_${solver_arch}.a"
cmake --build "$dir/build" --parallel 2
ctest --test-dir "$dir/build" --output-on-failure -E acceptance
cmake --install "$dir/build"
"""


def build() -> None:
    """Build the offline stack: toolchain image, Rust nodes, C++ simulator, runtime image.

    The five steps the shell used, owned by the tooling so `ap-pnc build simple-sim`
    is the single entry point and no shell script is needed.
    """
    root = get_git_root()
    base = stack_root()
    source = project_path(_STACK_DIR)
    arch = native_arch()
    tool = f"ap-pnc-simple-toolchain:{arch}"
    image = stack_image()
    platform = f"linux/{arch}"
    environment = setup_env()

    def run(command: list[str], *, quiet: bool = False) -> None:
        subprocess.run(
            command,
            env=environment,
            check=True,
            capture_output=quiet,
            text=quiet,
        )

    run(["docker", "image", "inspect", f"simple-sim:{arch}"], quiet=True)
    run([
        "docker", "build", "--platform", platform, "--target", "toolchain",
        "-t", tool, "-f", str(source / "Dockerfile"), str(root),
    ])
    run([
        "docker", "run", "--rm", "--platform", platform, "--ipc=host",
        "--entrypoint", "/bin/bash",
        "-e", "AP_PNC_DIR=/workspace",
        "-e", "CARGO_HOME=/workspace/.artifacts/simple-stack/cache/cargo",
        "-v", f"{root}:/workspace:ro",
        "-v", f"{source}:/workspace/infra/sim_infra/simple_sim/stack:rw",
        "-v", f"{base}:/workspace/.artifacts/simple-stack/cache:rw",
        tool, "-c", RUST_STEPS,
    ])
    run([
        "docker", "run", "--rm", "--platform", platform, "--ipc=host",
        "--network", "none", "--entrypoint", "/bin/bash",
        "-e", "AP_PNC_DIR=/workspace",
        "-e", "GIT_CONFIG_COUNT=1",
        "-e", "GIT_CONFIG_KEY_0=safe.directory",
        "-e", "GIT_CONFIG_VALUE_0=/workspace",
        "-v", f"{root}:/workspace:ro",
        "-v", f"{base}:/workspace/.artifacts/simple-stack/cache:rw",
        tool, "-c", CPP_STEPS,
    ])
    run([
        "docker", "build", "--platform", platform, "--target", "runtime",
        "-t", image, "-f", str(source / "Dockerfile"), str(root),
    ])


def run_job(request: JobRequest | None = None, *, job: Path | None = None) -> Path:
    """Run one job and return its directory.

    Raises on a failed run, including the case the graph exits zero without a
    verification receipt: a graph exit alone is not evidence that the recorded
    window is complete.
    """
    request = request or JobRequest()
    request.validate()
    os.environ.setdefault("AP_PNC_DIR", str(get_git_root()))
    root = stack_root()
    runs = root / "runs"
    runs.mkdir(parents=True, exist_ok=True)
    if job is None:
        job = runs / f"{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"
    (job / "benchmark").mkdir(parents=True, exist_ok=True)
    (job / "config").mkdir(parents=True, exist_ok=True)

    bringup = project_path("core/bringup/config")
    for name in ("simple_sim.yaml", "planning.yaml", "nmpc.yaml"):
        shutil.copy2(bringup / name, job / "config" / name)
    if request.profile == "practical":
        extracted = artifact_path(".artifacts/vtol-matlab")
        shutil.copy2(extracted / "profile.yaml", job / "profile.yaml")
        shutil.copy2(extracted / "provenance.json", job / "upstream-provenance.json")

    _write_configuration(job, request)
    _write_metadata(job)

    result = _container(job, "dora", "run", f"{_JOB_IN_CONTAINER}/dataflow.yml")
    (job / "stack.log").write_text(result.stdout + result.stderr)
    receipts = list((job / "benchmark").glob("*/verified.json"))
    if result.returncode != 0 or len(receipts) != 1:
        raise RuntimeError(
            f"job {job.name} did not produce exactly one verification receipt "
            f"(exit {result.returncode}, {len(receipts)} receipts); see {job / 'stack.log'}"
        )
    return job


def export_job(job: Path) -> None:
    """Run the acceptance/export step over a finished job."""
    if not (job / "benchmark").is_dir():
        raise ValueError(f"{job} is not a job directory")
    shutil.copy2(project_path(_STACK_DIR) / "acceptance.py", job / "acceptance.py")
    result = _container(job, "python3", f"{_JOB_IN_CONTAINER}/acceptance.py")
    if result.returncode != 0:
        raise RuntimeError(f"acceptance failed for {job.name}: {result.stderr.strip()}")


def run_summary(job: Path) -> dict:
    """The run directory's manifest and metrics, as the table needs them."""
    run_dirs = sorted((job / "benchmark").glob("simple_sim_*"))
    if len(run_dirs) != 1:
        raise RuntimeError(f"{job} has {len(run_dirs)} simulator runs, expected one")
    run = run_dirs[0]
    return {
        "run_dir": run,
        "manifest": json.loads((run / "manifest.json").read_text()),
        "metrics": json.loads((run / "metrics.json").read_text()),
    }
