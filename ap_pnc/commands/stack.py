import os
import signal
import subprocess

import typer

from ap_pnc.env import get_git_root, source_ros2_workspace

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
    git_root = get_git_root()
    compose_file = git_root / "infra" / "sim_infra" / "gazebo" / "docker" / "docker-compose.yml"
    for svc in services:
        typer.echo(f"\n--- docker build {svc} ---")
        subprocess.run(
            [
                "docker",
                "compose",
                "-f",
                str(compose_file),
                "build",
                "--progress=plain",
                svc,
            ],
            check=True,
        )


@app.command(name="full-install")
def full_install(
    docker_all: bool = typer.Option(
        False,
        "--docker-all",
        help="Also build the slow images px4-simulator + qground_control "
        "(default: ap-pnc + nav_infra only)",
    ),
) -> None:
    """Bootstrap everything: venv, acados, OCP codegen + bundle, Docker images.

    Chains `ap-pnc install` -> `uv sync` (dev + extra) -> `gen-nmpc-lib` ->
    `docker compose build`, with a step-by-step log. Equivalent to the
    Quick-Start section of the README in one command.
    """
    from ap_pnc.commands import gen_nmpc_lib, install

    git_root = get_git_root()

    _step(1, "Python env + acados C libs (ap-pnc install)")
    install.install()

    _step(2, "ML/compute extras (uv sync --group dev --group extra)")
    subprocess.run(
        ["uv", "sync", "--group", "dev", "--group", "extra"],
        cwd=git_root,
        check=True,
    )

    _step(3, "acados OCP codegen + libnmpc_bundle.a (gen-nmpc-lib)")
    gen_nmpc_lib.cocp()

    _step(4, "Docker images")
    services = ["ap-pnc", "nav_infra"]
    if docker_all:
        services = ["px4-simulator", "qground_control"] + services
    _build_docker_images(services)

    typer.echo("\nFull install complete.")


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
