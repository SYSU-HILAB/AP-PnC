from typing import Annotated

import typer

from tooling.docker_runtime import compose, compose_env, ensure_bundle, target_arch
from tooling.env import get_git_root

app = typer.Typer(help="Native-architecture Docker simulation (headless by default).")
GUI_SERVICES = {"desktop", "qground_control", "gazebo-gui"}
SERVICES = {"px4-simulator", "gcs-heartbeat", "nav_infra", "ap-pnc", *GUI_SERVICES}
Arch = Annotated[str, typer.Option("--arch", help="native | arm64 | amd64")]
Gui = Annotated[bool, typer.Option("--gui", help="Enable container desktop, QGC and Gazebo GUI")]
NavTarget = Annotated[str, typer.Option("--nav-target", help="sim or real image build target")]
SelectedServices = Annotated[list[str] | None, typer.Argument(help="Optional service names")]


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


@app.command()
def build(
    services: SelectedServices = None,
    arch: Arch = "native",
    gui: Gui = False,
    nav_target: NavTarget = "sim",
) -> None:
    """Build target-native images; generate a missing solver bundle in Docker."""
    selected, gui = _selection(services, gui)
    root = get_git_root()
    compose_env(root, arch, nav_target)
    if not selected or "ap-pnc" in selected:
        ensure_bundle(root, arch)
        _compose("build", "--progress=plain", "ros2-base", arch=arch, nav_target=nav_target)
    _compose("build", "--progress=plain", *selected, arch=arch, gui=gui, nav_target=nav_target)


@app.command()
def up(
    services: SelectedServices = None,
    arch: Arch = "native",
    gui: Gui = False,
    nav_target: NavTarget = "sim",
) -> None:
    """Start SITL + ROS2 without arming; --gui adds http://localhost:6088/vnc.html."""
    selected, gui = _selection(services, gui)
    _compose("up", "-d", *selected, arch=arch, gui=gui, nav_target=nav_target)


@app.command()
def down(arch: Arch = "native", nav_target: NavTarget = "sim") -> None:
    """Stop this Compose project, including its optional GUI services."""
    _compose("down", arch=arch, nav_target=nav_target)


@app.command()
def config(arch: Arch = "native", gui: Gui = False, nav_target: NavTarget = "sim") -> None:
    """Print resolved config without building or starting containers."""
    _compose("config", arch=arch, gui=gui, nav_target=nav_target)


@app.command()
def dev(arch: Arch = "native") -> None:
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
