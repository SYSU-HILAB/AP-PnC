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
    for name in ("c_nmpc", "px4_interface", "trajectory_server"):
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
    """Launch the full local stack (trajectory_server + px4_interface + c_nmpc).

    Cores are pinned on the last three CPUs, mirroring the legacy runall tool.
    Ctrl-C stops everything.
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
    what: str = typer.Argument(..., help="'pos_est' (pub /est_cmd) or 'go' (pub /mode)"),
) -> None:
    """Publish a one-shot Boolean trigger to a stack topic."""
    topics = {"pos_est": "/est_cmd", "go": "/mode"}
    topic = topics.get(what)
    if topic is None:
        typer.echo(f"Unknown trigger '{what}'. Choose from: {', '.join(topics)}.", err=True)
        raise typer.Exit(1)
    env = source_ros2_workspace()
    subprocess.run(
        ["ros2", "topic", "pub", topic, "std_msgs/msg/Bool", "data: true", "-1"],
        env=env,
        check=True,
    )
