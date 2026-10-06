import os
import signal
import subprocess
from typing import Annotated

import typer

from tooling.env import source_ros2_workspace

app = typer.Typer(help="Local ROS2 stack lifecycle (no containers).")

PIDS: list[int] = []


def _cleanup(*_: int) -> None:
    typer.echo("Stopping all stack processes...")
    for pid in PIDS:
        if pid > 0:
            subprocess.run(["kill", str(pid)], stderr=subprocess.DEVNULL)
    for name in ("component_container", "nmpc_node", "planner_node", "px4ctrl"):
        subprocess.run(["pkill", "-f", name], stderr=subprocess.DEVNULL)
    for pid in PIDS:
        if pid > 0:
            subprocess.run(["kill", "-9", str(pid)], stderr=subprocess.DEVNULL)


def _launch(env: dict[str, str], cores: str, *args: str) -> None:
    cmd = ["taskset", "-c", cores, "ros2", "launch", *args]
    proc = subprocess.Popen(cmd, env=env)
    PIDS.append(proc.pid)


def _step(n: int, title: str) -> None:
    typer.echo("\n" + "=" * 72)
    typer.echo(f"STEP {n}: {title}")
    typer.echo("=" * 72)


def _build_docker_images(services: list[str]) -> None:
    from tooling.commands import docker

    docker.build(services=services)


@app.command(name="full-install")
def full_install(
    docker_all: Annotated[
        bool, typer.Option("--docker-all", help="Include PX4 and the optional container GUI")
    ] = False,
    arch: Annotated[str, typer.Option("--arch", help="native | arm64 | amd64")] = "native",
) -> None:
    """Build the Docker toolchain and images without installing ROS/acados on the host."""
    from tooling.commands import docker

    services = None if docker_all else ["ap-pnc", "nav_infra"]
    docker.build(services=services, arch=arch, gui=docker_all)
    typer.echo("\nDocker installation complete (flight acceptance is a separate step).")


@app.command()
def start() -> None:
    """Launch the local research stack (planner + nmpc control core).

    Runs both as composable components inside one
    `component_container_mt` process (single launch); the base services
    (mavros + px4ctrl + LIO) run inside the nav_infra container. The
    container is pinned to the last two CPUs. Ctrl-C stops everything.
    """
    env = source_ros2_workspace()

    ncpu = os.cpu_count() or 0
    if ncpu < 2:
        typer.echo("At least 2 CPU cores are required to pin the stack.", err=True)
        raise typer.Exit(1)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)
    typer.echo(f"Detected {ncpu} CPU cores, pinning stack to cores {ncpu - 2}-{ncpu - 1}")

    _launch(env, f"{ncpu - 2},{ncpu - 1}", "bringup", "bringup.launch.py")

    typer.echo(f"All processes started (PIDs: {PIDS}). Press CTRL-C to stop.")
    try:
        while True:
            signal.pause()
    except KeyboardInterrupt:
        _cleanup()


@app.command()
def trigger(
    what: str = typer.Argument(..., help="'go' (pass-through on) or 'back' (pass-through off)"),
) -> None:
    """Synchronously toggle the px4ctrl PASS_THROUGH state (NMPC handover)."""
    actions = {"go": "true", "back": "false"}
    if what not in actions:
        typer.echo(f"Unknown trigger '{what}'. Choose from: {', '.join(actions)}.", err=True)
        raise typer.Exit(1)
    env = source_ros2_workspace()
    subprocess.run(
        [
            "ros2",
            "service",
            "call",
            "/px4ctrl/toggle_pass_through",
            "std_srvs/srv/SetBool",
            f"{{data: {actions[what]}}}",
        ],
        env=env,
        check=True,
    )
