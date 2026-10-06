import json
import os
import subprocess
import time
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console

from tooling.docker_runtime import compose, compose_env, ensure_bundle
from tooling.env import get_git_root

app = typer.Typer(help="Lockstep tracking runs and K3s simulation management")
console = Console()


def _run_kubectl(
    *args: str, check: bool = True, capture: bool = False
) -> subprocess.CompletedProcess:
    cmd = ["kubectl", *args]
    return subprocess.run(cmd, check=check, capture_output=capture, text=True)


def _ensure_k3s_deployed(namespace: str = "sim") -> None:
    """Ensure K3s simulation stack (ConfigMap, RBAC, Dispatcher, Deployments) is deployed."""
    res = _run_kubectl("get", "cm", "sim-config", "-n", namespace, check=False, capture=True)
    if res.returncode != 0:
        git_root = get_git_root()
        manifest = git_root / "infra/sim_k3s/all-in-one.yaml"
        console.print(f"[bold cyan]Deploying K3s simulation stack from {manifest}...[/bold cyan]")
        _run_kubectl("apply", "-f", str(manifest))
        time.sleep(1)


def _switch_sim(target: str, namespace: str = "sim") -> None:
    """Update SIM_TYPE in ConfigMap 'sim-config'."""
    _ensure_k3s_deployed(namespace)
    patch_payload = json.dumps({"data": {"SIM_TYPE": target}})
    _run_kubectl("patch", "cm", "sim-config", "-n", namespace, "-p", patch_payload)
    console.print(
        f"[bold green]✓ ConfigMap 'sim-config' set to SIM_TYPE='{target}'. "
        f"K3s dispatcher will reconcile automatically.[/bold green]"
    )


@app.command()
def simple(
    namespace: str = typer.Option("sim", "--namespace", "-n", help="Kubernetes namespace"),
    follow: bool = typer.Option(False, "--follow", "-f", help="Follow logs after starting"),
) -> None:
    """Start the tracking-only lockstep simulator via K3s ConfigMap (initially paused)."""
    console.print("[bold cyan]Switching simulation to simple-sim via K3s ConfigMap...[/bold cyan]")
    _switch_sim("simple-sim", namespace=namespace)
    time.sleep(1)
    status(namespace=namespace)
    if follow:
        logs(follow=True, namespace=namespace)


@app.command()
def track(
    config: Annotated[
        Path | None, typer.Option("--config", help="Absolute simple_sim YAML override")
    ] = None,
    controller: Annotated[str | None, typer.Option("--controller", help="nmpc or se3")] = None,
) -> None:
    """Run the installed lockstep tracking executable headlessly, without ROS/K3s."""
    root = get_git_root()
    executable = root / ".artifacts/colcon/install/lib/simple_sim/simple_sim_run"
    if not executable.is_file():
        raise typer.BadParameter(
            "simple_sim_run not installed; run ap-pnc build simple-sim on Linux"
        )
    if config is not None and not config.is_absolute():
        raise typer.BadParameter("--config must be an absolute path")
    if controller is not None and controller not in ("nmpc", "se3"):
        raise typer.BadParameter("--controller must be nmpc or se3")
    command = [str(executable)]
    if config is not None:
        command.extend(["--config", str(config)])
    if controller is not None:
        command.extend(["--controller", controller])
    subprocess.run(command, env={**os.environ, "AP_PNC_DIR": str(root)}, check=True)


@app.command()
def gazebo(
    namespace: str = typer.Option("sim", "--namespace", "-n", help="Kubernetes namespace"),
    follow: bool = typer.Option(False, "--follow", "-f", help="Follow logs after starting"),
) -> None:
    """Start Gazebo layer (PX4-VTOL SITL + Gazebo Garden) via K3s ConfigMap."""
    console.print("[bold cyan]Switching simulation to gazebo via K3s ConfigMap...[/bold cyan]")
    _switch_sim("gazebo", namespace=namespace)
    time.sleep(1)
    status(namespace=namespace)
    if follow:
        logs(follow=True, namespace=namespace)


@app.command()
def stop(
    namespace: str = typer.Option("sim", "--namespace", "-n", help="Kubernetes namespace"),
) -> None:
    """Stop all active simulation instances via K3s ConfigMap (scale to 0)."""
    console.print("[bold yellow]Stopping simulations via K3s ConfigMap...[/bold yellow]")
    _switch_sim("stop", namespace=namespace)
    time.sleep(1)
    status(namespace=namespace)


@app.command()
def down(
    namespace: str = typer.Option("sim", "--namespace", "-n", help="Kubernetes namespace"),
) -> None:
    """Alias for 'stop'."""
    stop(namespace=namespace)


@app.command()
def status(
    namespace: str = typer.Option("sim", "--namespace", "-n", help="Kubernetes namespace"),
) -> None:
    """Show current K3s ConfigMap selection and simulation Pod status."""
    cm_proc = _run_kubectl(
        "get",
        "cm",
        "sim-config",
        "-n",
        namespace,
        "-o",
        "jsonpath={.data.SIM_TYPE}",
        check=False,
        capture=True,
    )
    sim_type = cm_proc.stdout.strip() if cm_proc.returncode == 0 else "[not deployed]"

    console.print(
        f"\n[bold]Current ConfigMap SIM_TYPE:[/bold] [bold yellow]{sim_type}[/bold yellow]"
    )
    console.print("[bold]AP-PnC Workloads in K3s:[/bold]")
    _run_kubectl(
        "get",
        "deployments,pods",
        "-n",
        namespace,
        "-l",
        "app.kubernetes.io/part-of=ap-pnc",
        check=False,
    )


@app.command()
def logs(
    follow: bool = typer.Option(True, "--follow/--no-follow", "-f/-F", help="Follow log output"),
    namespace: str = typer.Option("sim", "--namespace", "-n", help="Kubernetes namespace"),
) -> None:
    """Stream logs of the currently running simulation Pod."""
    pods_proc = _run_kubectl(
        "get",
        "pods",
        "-n",
        namespace,
        "-l",
        "app.kubernetes.io/part-of=ap-pnc",
        "-o",
        "jsonpath={range .items[*]}{.metadata.name}{' '}{.status.phase}{'\\n'}{end}",
        check=False,
        capture=True,
    )
    running_sim_pod = None
    for line in pods_proc.stdout.strip().splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[1] == "Running" and "dispatcher" not in parts[0]:
            running_sim_pod = parts[0]
            break

    if not running_sim_pod:
        console.print(
            "[yellow]No active simulation pod is currently running. Streaming sim-dispatcher logs:[/yellow]"
        )
        args = ["logs", "-n", namespace, "deployment/sim-dispatcher"]
        if follow:
            args.append("-f")
        _run_kubectl(*args)
        return

    console.print(f"[bold cyan]Streaming logs from {running_sim_pod}...[/bold cyan]")
    args = ["logs", "-n", namespace, running_sim_pod]
    if follow:
        args.append("-f")
    _run_kubectl(*args)


@app.command()
def build(
    target: Annotated[str, typer.Option("--target", "-t", help="simple, gazebo, or all")] = "all",
    arch: Annotated[str, typer.Option("--arch", help="native | arm64 | amd64")] = "native",
    gui: Annotated[bool, typer.Option("--gui", help="Build the optional Gazebo desktop")] = False,
    nav_target: Annotated[str, typer.Option("--nav-target", help="sim or real nav image")] = "sim",
) -> None:
    """Build images for one selected Linux target, using isolated solver generation."""
    if target not in ("simple", "simple-sim", "gazebo", "all"):
        raise typer.BadParameter("--target must be simple, gazebo or all")
    root = get_git_root()
    compose_env(root, arch, nav_target)
    if target in ("simple", "simple-sim", "all"):
        ensure_bundle(root, arch)
        compose(
            root,
            root / "infra/sim_infra/simple_sim/docker/docker-compose.yml",
            ["build", "--progress=plain", "ros2-base"],
            arch=arch,
        )
        compose(
            root,
            root / "infra/sim_infra/simple_sim/docker/docker-compose.yml",
            ["build", "--progress=plain", "simple-sim"],
            arch=arch,
        )
    if target in ("gazebo", "all"):
        from tooling.commands import docker

        docker.build(arch=arch, gui=gui, nav_target=nav_target)
