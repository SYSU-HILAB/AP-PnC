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
        print(f"Error: Python interpreter not found!", file=sys.stderr)
        print(f"", file=sys.stderr)
        print(f"Git Root detected: {git_root}", file=sys.stderr)
        print(f"Searched for venv at: {git_root}/{{.acados_venv,.venv,venv,env}}/bin/python", file=sys.stderr)
        print(f"", file=sys.stderr)
        print(f"To fix this, run one of the following:", file=sys.stderr)
        print(f"  just install-venv    # Create virtual environment", file=sys.stderr)
        print(f"  just install-all     # Full installation with acados", file=sys.stderr)
        print(f"", file=sys.stderr)
        print(f"Or set AP_PNC_DIR override:", file=sys.stderr)
        print(f"  AP_PNC_DIR=/custom/path ./pyp", file=sys.stderr)
        return 1

    # Execute python with all arguments passed to this script
    # If no args, run interactive shell
    args = sys.argv[1:] if len(sys.argv) > 1 else []

    try:
        result = subprocess.run(
            [str(python_bin)] + args,
            env=os.environ.copy(),
            check=False
        )
        return result.returncode
    except FileNotFoundError:
        print(f"Error: {python_bin} does not exist or is not executable", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130  # Standard exit code for SIGINT


if __name__ == "__main__":
    sys.exit(main())
