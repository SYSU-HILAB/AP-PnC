import os
import subprocess
from pathlib import Path


def get_git_root() -> Path:
    """Find the git repository root, handling worktrees via commondir."""
    ap_pnc_dir = os.environ.get("AP_PNC_DIR", "")
    if ap_pnc_dir:
        return Path(ap_pnc_dir).resolve()

    try:
        result = subprocess.run(
            ["git", "rev-parse", "--git-dir"],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            return Path.cwd()

        git_dir = Path(result.stdout.strip())
        if git_dir.is_absolute():
            git_file = git_dir
        else:
            git_file = Path.cwd() / git_dir

        if "/worktrees/" in str(git_file):
            commondir_file = git_file / "commondir"
            if commondir_file.exists():
                commondir = commondir_file.read_text().strip()
                original_git_dir = (git_file / commondir).resolve()
                return original_git_dir.parent

        return git_file.parent.resolve()
    except (subprocess.SubprocessError, FileNotFoundError):
        return Path.cwd()


def source_ros2_env() -> dict[str, str]:
    """Source ROS2 setup.bash via subprocess and return merged environment."""
    ros_paths = ["/opt/ros/humble", "/opt/ros/iron", "/opt/ros/jazzy"]
    env = os.environ.copy()

    for ros_path in ros_paths:
        setup_file = Path(ros_path) / "setup.bash"
        if setup_file.exists():
            result = subprocess.run(
                ["bash", "-c", f"source {setup_file} && env"],
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                for line in result.stdout.splitlines():
                    if "=" in line:
                        key, _, value = line.partition("=")
                        env[key] = value
                return env
    return env


def source_ros2_workspace() -> dict[str, str]:
    """Source ROS2 plus the colcon workspace install (.artifacts/colcon/install)."""
    git_root = get_git_root()
    env = source_ros2_env()
    workspace_setup = git_root / ".artifacts" / "colcon" / "install" / "setup.bash"
    if workspace_setup.exists():
        result = subprocess.run(
            ["bash", "-c", f"source {workspace_setup} && env"],
            capture_output=True,
            text=True,
            env=env,
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                if "=" in line:
                    key, _, value = line.partition("=")
                    env[key] = value
    return env


def setup_env(ros: bool = False) -> None:
    """Configure os.environ with acados paths and optionally ROS2.

    ACADOS_SOURCE_DIR points at the staged install tree (.artifacts/acados),
    which provides include/, lib/, bin/t_renderer for acados_template codegen.
    ACADOS_PYTHON_INTERFACE_PATH falls back to the source submodule for the
    Python interface, so mpc_framework stays a pure source checkout.
    """
    git_root = get_git_root()
    os.environ.setdefault("AP_PNC_DIR", str(git_root))
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(git_root / ".artifacts" / "acados"))
    os.environ.setdefault(
        "ACADOS_PYTHON_INTERFACE_PATH",
        str(git_root / "mpc_framework" / "interfaces" / "acados_template" / "acados_template"),
    )

    lib_path = str(git_root / ".artifacts" / "acados" / "lib")
    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    if lib_path not in existing_ld:
        os.environ["LD_LIBRARY_PATH"] = f"{lib_path}:{existing_ld}"

    pythonpath = os.environ.get("PYTHONPATH", "")
    git_root_str = str(git_root)
    bindings_dir = str(git_root / ".artifacts" / "bindings")
    parts = [p for p in (git_root_str, bindings_dir) if p not in pythonpath]
    if parts:
        os.environ["PYTHONPATH"] = ":".join(parts) + (f":{pythonpath}" if pythonpath else "")

    if ros:
        ros_env = source_ros2_env()
        os.environ.update(ros_env)
