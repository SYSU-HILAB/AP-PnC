import os
import signal
import subprocess

import typer

from ap_pnc.env import source_ros2_workspace

app = typer.Typer(help="Local ROS2 stack lifecycle (no containers).")

PIDS: list[int] = []


def _cleanup(*_: int) -> None:
    typer.echo("Stopping all stack processes...")
    for pid in PIDS:
        if pid > 0:
            subprocess.run(["kill", str(pid)], stderr=subprocess.DEVNULL)
    for name in ("nmpc", "planner", "px4ctrl"):
        subprocess.run(["pkill", "-f", name], stderr=subprocess.DEVNULL)
    for pid in PIDS:
        if pid > 0:
            subprocess.run(["kill", "-9", str(pid)], stderr=subprocess.DEVNULL)


def _launch(env: dict[str, str], core: int, *args: str) -> None:
    cmd = ["taskset", "-c", str(core), "ros2", "launch", *args]
    proc = subprocess.Popen(cmd, env=env)
    PIDS.append(proc.pid)


@app.command()
def start() -> None:
    """Launch the local research stack (planner + nmpc control core).

    The base services (mavros + px4ctrl + LIO) run inside the nav_infra
    container; bring them up with the gazebo compose file (see the
    ap-pnc-docker skill). Cores are pinned on the last two CPUs, mirroring
    the legacy runall tool. Ctrl-C stops everything.
    """
    env = source_ros2_workspace()

    ncpu = os.cpu_count() or 0
    if ncpu < 2:
        typer.echo("At least 2 CPU cores are required to pin the stack.", err=True)
        raise typer.Exit(1)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)
    typer.echo(f"Detected {ncpu} CPU cores, pinning stack to cores {ncpu - 2}-{ncpu - 1}")

    _launch(env, ncpu - 2, "bringup", "planning.launch.py")
    _launch(env, ncpu - 1, "bringup", "bringup.launch.py")

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
            "ros2", "service", "call", "/px4ctrl/toggle_pass_through",
            "std_srvs/srv/SetBool", f"{{data: {actions[what]}}}",
        ],
        env=env,
        check=True,
    )
