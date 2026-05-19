import subprocess
import sys

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()


@app.command()
def viz(
    playback_dt: float = typer.Option(0.05, help="Playback delta time [s]"),
    rviz_config: str = typer.Option(
        "feature/lu_mpc/ros/config/ref_trajs.rviz", help="RViz config file path"
    ),
) -> None:
    """Launch reference trajectory publisher + RViz2."""
    setup_env(ros=True)
    git_root = get_git_root()
    script = git_root / "test" / "lu_mpc" / "ros" / "user_test_ref_traj_viz.py"
    subprocess.run(
        [sys.executable, str(script), "--playback-dt", str(playback_dt), "--rviz-config", rviz_config],
        check=True,
    )


@app.command(name="viz-tracking")
def viz_tracking(
    rviz_config: str = typer.Option(
        "feature/lu_mpc/ros/config/nmpc_viz.rviz", help="RViz config file path"
    ),
) -> None:
    """Launch tracking visualization + RViz2."""
    setup_env(ros=True)
    git_root = get_git_root()
    script = git_root / "test" / "lu_mpc" / "ros" / "user_test_traj_tracking_viz.py"
    subprocess.run(
        [sys.executable, str(script), "--rviz-config", rviz_config],
        check=True,
    )


@app.command(name="viz-elmpc")
def viz_elmpc(
    rviz_config: str = typer.Option(
        "feature/lu_mpc/ros/config/nmpc_viz.rviz", help="RViz config file path"
    ),
) -> None:
    """Launch error-state MPC tracking visualization + RViz2."""
    setup_env(ros=True)
    git_root = get_git_root()
    script = git_root / "test" / "lu_mpc" / "ros" / "user_test_traj_tracking_elmpc_viz.py"
    subprocess.run(
        [sys.executable, str(script), "--rviz-config", rviz_config, "--hover"],
        check=True,
    )


@app.command()
def odom(
    rviz_config: str = typer.Option(
        "feature/lu_mpc/ros/config/plotjuggler_odometry_xyz.xml", help="PlotJuggler config file path"
    ),
) -> None:
    """Launch PlotJuggler for odometry XYZ."""
    setup_env(ros=True)
    subprocess.run(["ros2", "run", "plotjuggler", "plotjuggler", "-l", rviz_config], check=True)
