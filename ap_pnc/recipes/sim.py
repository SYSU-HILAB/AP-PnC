import json
import subprocess
import time

import typer
from rich.console import Console

from ap_pnc.env import get_git_root

app = typer.Typer(help="Simulation management (K3s ConfigMap-driven coarse-grained commands)")
console = Console()


def _run_kubectl(*args: str, check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    cmd = ["kubectl", *args]
    return subprocess.run(cmd, check=check, capture_output=capture, text=True)


def _ensure_k3s_deployed(namespace: str = "sim") -> None:
    """Ensure K3s simulation stack (ConfigMap, RBAC, Dispatcher, Deployments) is deployed."""
    res = _run_kubectl("get", "cm", "sim-config", "-n", namespace, check=False, capture=True)
    if res.returncode != 0:
        git_root = get_git_root()
        manifest = git_root / "simulations/k3s/all-in-one.yaml"
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
    """Start simple-sim layer (lightweight LiDAR + aerodynamics simulator) via K3s ConfigMap."""
    console.print("[bold cyan]Switching simulation to simple-sim via K3s ConfigMap...[/bold cyan]")
    _switch_sim("simple-sim", namespace=namespace)
    time.sleep(1)
    status(namespace=namespace)
    if follow:
        logs(follow=True, namespace=namespace)


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

    console.print(f"\n[bold]Current ConfigMap SIM_TYPE:[/bold] [bold yellow]{sim_type}[/bold yellow]")
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
    target: str = typer.Option("all", "--target", "-t", help="Target: simple, gazebo, or all"),
) -> None:
    """Rebuild simulation Docker images."""
    git_root = get_git_root()
    if target in ("simple", "simple-sim", "all"):
        console.print("[bold cyan]Building simple-sim Docker image...[/bold cyan]")
        subprocess.run(
            [
                "docker",
                "compose",
                "-f",
                str(git_root / "simulations/simple_sim/docker/docker-compose.yml"),
                "build",
            ],
            check=True,
        )
    if target in ("gazebo", "all"):
        console.print("[bold cyan]Building Gazebo Docker images...[/bold cyan]")
        subprocess.run(
            [
                "docker",
                "compose",
                "-f",
                str(git_root / "simulations/gazebo/docker/docker-compose.yml"),
                "build",
            ],
            check=True,
        )


@app.command()
def pcd() -> None:
    """Dump one LiDAR frame from simple_sim and view it (open3d via uv)."""
    git_root = get_git_root()
    # If running in K3s, exec into pod, else run locally
    pods_proc = _run_kubectl(
        "get",
        "pods",
        "-n",
        "sim",
        "-l",
        "app.kubernetes.io/name=ap-pnc-simple-sim",
        "-o",
        "jsonpath={.items[0].metadata.name}",
        check=False,
        capture=True,
    )
    sim_pod = pods_proc.stdout.strip()
    if sim_pod:
        console.print(f"[bold cyan]Dumping PCD frame from K3s pod '{sim_pod}'...[/bold cyan]")
        _run_kubectl(
            "exec",
            "-n",
            "sim",
            sim_pod,
            "--",
            "bash",
            "-c",
            "source /workspace/install/setup.bash && ros2 run simple_sim simple_sim_node "
            "--ros-args -p dump_pcd_path:=/tmp/frame.pcd & "
            "PID=$!; sleep 3; kill $PID 2>/dev/null; wait 2>/dev/null; true",
        )
        local_pcd = git_root / ".artifacts/simple-sim/frame.pcd"
        local_pcd.parent.mkdir(parents=True, exist_ok=True)
        _run_kubectl("cp", f"sim/{sim_pod}:/tmp/frame.pcd", str(local_pcd))
        viz_script = git_root / "ap_pnc/tools/viz_pcd.py"
        subprocess.run(
            ["uv", "run", "--script", str(viz_script), str(local_pcd)], check=True
        )
    else:
        console.print("[yellow]No ap-pnc-simple-sim pod running. Starting simple-sim first...[/yellow]")
        simple()
