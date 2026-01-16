#!/usr/bin/env python3
"""
Python wrapper for AP-PnC project.

This script locates and executes the project's Python interpreter from the
.acados_venv virtual environment, with full git worktree support.

Usage:
    ./pyp [args]          # Run python with args
    ./pyp -m module       # Run module
    ./pyp script.py       # Run Python script
"""

import os
import sys
import subprocess
from pathlib import Path


# === ROS2 Setup ===
def setup_ros2_environment(env: dict) -> None:
    """
    Source ROS2 setup.bash and update environment variables.

    Args:
        env: Dictionary of environment variables to update in-place
    """
    # Common ROS2 installation paths
    ros_paths = [
        "/opt/ros/humble",
        "/opt/ros/iron",
        "/opt/ros/jazzy",
    ]

    for ros_path in ros_paths:
        setup_file = Path(ros_path) / "setup.bash"
        if setup_file.exists():
            try:
                # Source the setup.bash script and capture environment variables
                result = subprocess.run(
                    ["bash", "-c", f"source {setup_file} && env"],
                    capture_output=True,
                    text=True,
                    check=False
                )
                if result.returncode == 0:
                    # Parse the output and update the environment
                    for line in result.stdout.splitlines():
                        if "=" in line:
                            key, value = line.split("=", 1)
                            env[key] = value
                    return  # Successfully sourced ROS2
            except (subprocess.SubprocessError, FileNotFoundError):
                pass


# === ANSI Color Codes ===
class Colors:
    GREEN = "\033[92m"      # Success green
    YELLOW = "\033[93m"     # Warning yellow
    BLUE = "\033[94m"       # Info blue
    RED = "\033[91m"        # Error red
    BOLD = "\033[1m"        # Bold text
    RESET = "\033[0m"       # Reset formatting
    CHECKMARK = "\u2713"    # ✓ symbol


def color_print(text: str, color: str = Colors.RESET) -> None:
    """Print text with ANSI color formatting."""
    print(f"{color}{text}{Colors.RESET}")


def find_git_root() -> Path:
    """
    Find the git repository root, handling both normal repos and worktrees.

    Returns:
        Path: The git root directory
    """
    # Check for AP_PNC_DIR override (environment variable)
    ap_pnc_dir = os.environ.get("AP_PNC_DIR", "")
    if ap_pnc_dir:
        return Path(ap_pnc_dir).resolve()

    # Try to find git directory
    cwd = Path.cwd()

    # Try git rev-parse to find git dir
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--git-dir"],
            cwd=cwd,
            capture_output=True,
            text=True,
            check=False
        )
        if result.returncode == 0:
            git_file = Path(result.stdout.strip())

            # Check if we're in a worktree (gitdir contains /worktrees/)
            if "/worktrees/" in str(git_file):
                # Read commondir to find original repo's .git location
                commondir_file = git_file / "commondir"
                if commondir_file.exists():
                    commondir = commondir_file.read_text().strip()
                    original_git_dir = (git_file / commondir).resolve()
                    # The repo root is the parent of the .git directory
                    return original_git_dir.parent
            else:
                # Normal repo - .git is a directory at root
                return git_file.parent.resolve()
    except (subprocess.SubprocessError, FileNotFoundError):
        pass

    # Not in git repo - use script directory as fallback
    return Path(__file__).parent.resolve()


def find_python_interpreter(git_root: Path) -> Path | None:
    """
    Find the Python interpreter in the virtual environment.

    Args:
        git_root: The git repository root directory

    Returns:
        Path to the Python interpreter, or None if not found
    """
    venv_names = [".acados_venv", ".venv", "venv", "env"]

    for venv_name in venv_names:
        python_bin = git_root / venv_name / "bin" / "python"
        if python_bin.exists():
            return python_bin

    return None


def main() -> int:
    """Main entry point."""
    git_root = find_git_root()
    python_bin = find_python_interpreter(git_root)

    if python_bin is None:
        color_print(f"Error: Python interpreter not found!", Colors.RED + Colors.BOLD)
        print()
        color_print(f"Git Root detected: {git_root}", Colors.YELLOW)
        color_print(f"Searched for venv at: {git_root}/{{.acados_venv,.venv,venv,env}}/bin/python", Colors.YELLOW)
        print()
        color_print(f"To fix this, run one of the following:", Colors.BOLD)
        color_print(f"  just install-venv    # Create virtual environment", Colors.GREEN)
        color_print(f"  just install-all     # Full installation with acados", Colors.GREEN)
        print()
        color_print(f"Or set AP_PNC_DIR override:", Colors.BOLD)
        color_print(f"  AP_PNC_DIR=/custom/path ./pyp", Colors.GREEN)
        return 1

    # Display colored status messages
    git_root_str = str(git_root)
    venv_name = python_bin.parent.parent.name  # Extracts ".acados_venv" from path

    color_print(f"AP-PnC Environment Loader", Colors.BOLD + Colors.BLUE)
    color_print(f"  Git Root: {git_root_str}", Colors.BLUE)
    color_print(f"  Using: {venv_name} {Colors.CHECKMARK}", Colors.GREEN)
    print()  # Blank line for separation

    # Execute python with all arguments passed to this script
    # If no args, run interactive shell
    args = sys.argv[1:] if len(sys.argv) > 1 else []

    # Setup environment including ROS2 if available
    env = os.environ.copy()
    setup_ros2_environment(env)

    # Add ACADOS library path to LD_LIBRARY_PATH
    acados_lib_path = str(git_root / "mpc_framework" / "lib")
    ld_library_path = env.get("LD_LIBRARY_PATH", "")
    if acados_lib_path not in ld_library_path:
        env["LD_LIBRARY_PATH"] = f"{acados_lib_path}:{ld_library_path}"

    try:
        os.execvpe(str(python_bin), [str(python_bin)] + args, env)
    except FileNotFoundError:
        print(f"Error: {python_bin} does not exist or is not executable", file=sys.stderr)
        return 1
    except PermissionError:
        print(f"Error: {python_bin} is not executable", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130  # Standard exit code for SIGINT


if __name__ == "__main__":
    sys.exit(main())
