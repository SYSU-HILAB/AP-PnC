"""Host-native Docker targeting; project resources never depend on cwd."""

import os
import platform
import subprocess
from collections.abc import Sequence
from pathlib import Path

import typer

from tooling.env import get_git_root

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
    return {
        **os.environ,
        "AP_PNC_DIR": str(root),
        "AP_PNC_ARCH": target_arch(arch),
        "AP_PNC_NAV_TARGET": nav_target,
    }


def compose(
    root: Path,
    compose_file: Path,
    args: Sequence[str],
    *,
    arch: str = "native",
    gui: bool = False,
    nav_target: str = "sim",
) -> None:
    if not compose_file.is_absolute():
        raise ValueError("Compose file must be absolute")
    command = ["docker", "compose", "-f", str(compose_file)]
    if args and args[0] == "build":
        command += ["--profile", "build"]
    if "--progress=plain" in args:
        command += ["--progress", "plain"]
        args = [arg for arg in args if arg != "--progress=plain"]
    if gui or (args and args[0] == "down"):
        command += ["--profile", "gui"]
    subprocess.run(
        [*command, *args],
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
