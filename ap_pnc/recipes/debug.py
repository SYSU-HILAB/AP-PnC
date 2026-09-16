import shutil
import subprocess

import typer

from ap_pnc.env import get_git_root, source_ros2_workspace

app = typer.Typer(help="Interactive debugging helpers.")

TOPICS = (
    "/cur_state",
    "/cur_state_odom",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/vehicle_command",
    "/fmu/in/vehicle_thrust_acc_setpoint",
    "/fmu/in/vehicle_visual_odometry",
    "/fmu/in/vel_pitch_notification",
    "/fmu/out/timesync_status",
    "/fmu/out/vehicle_acceleration",
    "/fmu/out/vehicle_angular_velocity",
    "/fmu/out/vehicle_control_mode",
    "/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_thrust_setpoint",
    "/fmu/out/vehicle_torque_setpoint",
    "/mpc_controls",
    "/parameter_events",
    "/ref_can_set",
    "/ref_trajectory_bullet",
    "/rosout",
)


@app.command()
def echo() -> None:
    """Fuzzy-pick a ROS2 topic (fzf) and echo it."""
    if not shutil.which("fzf"):
        typer.echo("fzf not found. Install it first (e.g. pacman -S fzf).", err=True)
        raise typer.Exit(1)

    fzf = subprocess.run(
        ["fzf", "--prompt=Select a topic: "],
        input="\n".join(TOPICS),
        capture_output=True,
        text=True,
    )
    topic = fzf.stdout.strip()
    if not topic:
        typer.echo("No topic selected.")
        return
    typer.echo(f"Selected topic: {topic}")
    env = source_ros2_workspace()
    subprocess.run(["ros2", "topic", "echo", topic], env=env, check=True)


@app.command()
def pcd(
    file: str = typer.Argument(..., help="PCD file dumped by simple_sim"),
    info: bool = typer.Option(False, "--info", help="Print stats instead of opening a window"),
) -> None:
    """Visualize a simple_sim ASCII PCD with open3d (isolated PEP 723 deps)."""
    script = get_git_root() / "ap_pnc" / "tools" / "viz_pcd.py"
    args = ["uv", "run", "--script", str(script), file]
    if info:
        args.append("--info")
    subprocess.run(args, check=True)
