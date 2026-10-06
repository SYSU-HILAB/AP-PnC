"""Host-native Docker targeting; project resources never depend on cwd."""

import os
import platform
import re
import subprocess
from collections.abc import Sequence
from pathlib import Path

import typer

from tooling.env import artifact_path, get_git_root

ARCH_ALIASES = {
    "arm64": "arm64",
    "aarch64": "arm64",
    "amd64": "amd64",
    "x86_64": "amd64",
}
SOLVER_ARCH = {"arm64": "aarch64", "amd64": "x86_64"}


def target_arch(value: str = "native") -> str:
    machine = platform.machine().lower() if value == "native" else value.lower()
    try:
        return ARCH_ALIASES[machine]
    except KeyError as error:
        raise typer.BadParameter("--arch must be native, arm64/aarch64 or amd64/x86_64") from error


def compose_env(root: Path, arch: str = "native", nav_target: str = "sim") -> dict[str, str]:
    if not root.is_absolute():
        raise ValueError("Project root must be absolute")
    if nav_target not in ("sim", "real"):
        raise typer.BadParameter("--nav-target must be sim or real")
    _ = target_arch(arch)  # unknown architectures are rejected before anything runs
    # The host-dependent values live in the generated override, not here: this
    # project defines one environment variable.
    return {**os.environ, "AP_PNC_DIR": str(root)}


# Host-dependent literals in the base Compose files. They carry plain defaults so
# the files stay valid on their own; the tooling rewrites them for the host and
# writes the differing keys as an override. No variable is involved, which is why
# AP_PNC_DIR remains the project's only environment variable.
_HOST_LITERALS: tuple[tuple[str, str], ...] = (
    (r"linux/arm64", "linux/{arch}"),
    (r":arm64\b", ":{arch}"),
    (r"-sim\b", "-{nav_target}"),
    (r'"1000:1000"', '"{uid}:{gid}"'),
)


def _resolve(text: str, replacements: dict[str, str]) -> str:
    for pattern, template in _HOST_LITERALS:
        text = re.sub(pattern, template.format(**replacements), text)
    return text


def _diff(base, resolved, out: dict) -> None:
    """Collect the leaves where the resolved document differs from the base.

    Compose merges an override by key, so the override only carries the keys whose
    value depends on the host: platform, image tags, build target and container
    user.
    """
    if isinstance(base, dict) and isinstance(resolved, dict):
        for key, value in resolved.items():
            if key not in base:
                out[key] = value
            elif isinstance(base[key], dict) and isinstance(value, dict):
                child: dict = {}
                _diff(base[key], value, child)
                if child:
                    out[key] = child
            elif base[key] != value:
                out[key] = value
        return
    if isinstance(base, list) and isinstance(resolved, list) and len(base) == len(resolved):
        for index, value in enumerate(resolved):
            previous = base[index]
            if isinstance(previous, dict) and isinstance(value, dict):
                child = {}
                _diff(previous, value, child)
                if child:
                    out[index] = child
            elif previous != value:
                out[index] = value


def compose_command(
    root: Path,
    compose_file: Path,
    args: Sequence[str],
    *,
    arch: str = "native",
    gui: bool = False,
    nav_target: str = "sim",
) -> list[str]:
    """The Compose argv for one call, including the generated override."""
    if not compose_file.is_absolute():
        raise ValueError("Compose file must be absolute")
    override = compose_override(root, compose_file, arch, nav_target)
    command = ["docker", "compose", "-f", str(compose_file), "-f", str(override)]
    rest = [arg for arg in args if arg != "--progress=plain"]
    if args and args[0] == "build":
        command += ["--profile", "build"]
    if "--progress=plain" in args:
        command += ["--progress", "plain"]
    if gui or (rest and rest[0] == "down"):
        command += ["--profile", "gui"]
    return [*command, *rest]


def compose_override(
    root: Path, compose_file: Path, arch: str = "native", nav_target: str = "sim"
) -> Path:
    """Resolve the host-dependent Compose keys into an override file.

    The base files name one variable only, ``AP_PNC_DIR``, and carry literal
    defaults for everything host-dependent. This resolves those keys from the
    arguments, diffs them against the base and writes exactly the differing keys,
    so `AP_PNC_DIR` stays the project's only environment variable. Compose merges
    the result over the base by key.
    """
    import yaml  # - imported only on this path

    if not compose_file.is_absolute():
        raise ValueError("Compose file must be absolute")
    replacements = {
        "arch": target_arch(arch),
        "nav_target": nav_target,
        "uid": str(os.getuid()) if hasattr(os, "getuid") else "1000",
        "gid": str(os.getgid()) if hasattr(os, "getgid") else "1000",
    }
    text = compose_file.read_text()
    resolved_text = _resolve(text, replacements)
    override: dict = {}
    if resolved_text != text:
        _diff(yaml.safe_load(text), yaml.safe_load(resolved_text), override)
    out_dir = artifact_path(f".artifacts/docker/{replacements["arch"]}-{nav_target}")
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{compose_file.stem}.override.yml"
    header = (
        "# Generated by tooling.docker_runtime; do not edit.\n"
        "# Arch, nav target and container user are tooling arguments, not variables.\n"
    )
    path.write_text(header + yaml.safe_dump(override or {"services": {}}, sort_keys=False))
    return path


def compose(
    root: Path,
    compose_file: Path,
    args: Sequence[str],
    *,
    arch: str = "native",
    gui: bool = False,
    nav_target: str = "sim",
) -> None:
    subprocess.run(
        compose_command(
            root, compose_file, args, arch=arch, gui=gui, nav_target=nav_target
        ),
        env=compose_env(root, arch, nav_target),
        cwd=root,
        check=True,
    )


def ensure_bundle(root: Path, arch: str = "native") -> None:
    """Docker builds consume only the selected arch-tagged archive, never the host alias."""
    if not root.is_absolute():
        raise ValueError("Project root must be absolute")
    solver_arch = SOLVER_ARCH[target_arch(arch)]
    bundle = root / ".artifacts/nmpc_solver" / f"libnmpc_bundle_{solver_arch}.a"
    if not bundle.is_file():
        from tooling.commands.gen_nmpc_lib import cocp

        cocp(arch=solver_arch)
    if not bundle.is_file():
        raise RuntimeError(f"Solver builder did not produce {bundle}")


# ---------------------------------------------------------------------------
# Gazebo/ROS Compose layer.
#
# This was the `ap-pnc docker` command group. The group is gone: it was ahead of
# the roadmap. The functions stay because `sim` builds images through them and
# the tests exercise the selection rules, so they are an internal API now. The
# docstrings keep the CLI wording they had, minus the command line.
# ---------------------------------------------------------------------------
GUI_SERVICES = {"desktop", "qground_control", "gazebo-gui"}
SERVICES = {"px4-simulator", "gcs-heartbeat", "nav_infra", "ap-pnc", *GUI_SERVICES}

def _compose(*args: str, arch: str = "native", gui: bool = False, nav_target: str = "sim") -> None:
    root = get_git_root()
    compose(
        root,
        root / "infra/sim_infra/gazebo/docker/docker-compose.yml",
        args,
        arch=arch,
        gui=gui,
        nav_target=nav_target,
    )


def _selection(services: list[str] | None, gui: bool) -> tuple[list[str], bool]:
    selected = services or []
    if unknown := set(selected) - SERVICES:
        raise typer.BadParameter(f"Unknown services: {', '.join(sorted(unknown))}")
    return selected, gui or bool(set(selected) & GUI_SERVICES)


def build(
    services: list[str] | None = None,
    arch: str = "native",
    gui: bool = False,
    nav_target: str = "sim",
) -> None:
    """Build target-native images; generate a missing solver bundle in Docker."""
    selected, gui = _selection(services, gui)
    root = get_git_root()
    compose_env(root, arch, nav_target)
    if not selected or "ap-pnc" in selected:
        ensure_bundle(root, arch)
        _compose("build", "--progress=plain", "ros2-base", arch=arch, nav_target=nav_target)
    _compose("build", "--progress=plain", *selected, arch=arch, gui=gui, nav_target=nav_target)


def up(
    services: list[str] | None = None,
    arch: str = "native",
    gui: bool = False,
    nav_target: str = "sim",
) -> None:
    """Start SITL + ROS2 without arming; --gui adds http://localhost:6088/vnc.html."""
    selected, gui = _selection(services, gui)
    _compose("up", "-d", *selected, arch=arch, gui=gui, nav_target=nav_target)


def down(arch: str = "native", nav_target: str = "sim") -> None:
    """Stop this Compose project, including its optional GUI services."""
    _compose("down", arch=arch, nav_target=nav_target)


def config(arch: str = "native", gui: bool = False, nav_target: str = "sim") -> None:
    """Print resolved config without building or starting containers."""
    _compose("config", arch=arch, gui=gui, nav_target=nav_target)


def dev(arch: str = "native") -> None:
    """Open the matching Linux ROS2 development container (separate caches per arch)."""
    root = get_git_root()
    architecture = target_arch(arch)
    ensure_bundle(root, architecture)
    compose(
        root,
        root / "core/docker/docker-compose.yml",
        ["build", f"ros2-dev-{architecture}"],
        arch=architecture,
    )
    compose(
        root,
        root / "core/docker/docker-compose.yml",
        ["run", "--rm", f"ros2-dev-{architecture}"],
        arch=architecture,
    )
