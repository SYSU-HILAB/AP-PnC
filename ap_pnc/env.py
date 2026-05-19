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


def setup_env(ros: bool = False) -> None:
    """Configure os.environ with acados paths and optionally ROS2."""
    git_root = get_git_root()
    os.environ.setdefault("AP_PNC_DIR", str(git_root))
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(git_root / "mpc_framework"))

    lib_path = str(git_root / "mpc_framework" / "lib")
    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    if lib_path not in existing_ld:
        os.environ["LD_LIBRARY_PATH"] = f"{lib_path}:{existing_ld}"

    pythonpath = os.environ.get("PYTHONPATH", "")
    git_root_str = str(git_root)
    if git_root_str not in pythonpath:
        os.environ["PYTHONPATH"] = f"{git_root_str}:{pythonpath}"

    if ros:
        ros_env = source_ros2_env()
        os.environ.update(ros_env)
