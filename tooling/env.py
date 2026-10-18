import os
import subprocess
from pathlib import Path


def get_git_root() -> Path:
    """Resolve AP_PNC_DIR, or discover the checkout from this module, never cwd.

    Worktrees retain the project's shared-root convention via commondir. Installed
    wheels outside a checkout must supply AP_PNC_DIR explicitly.
    """
    configured = os.environ.get("AP_PNC_DIR")
    if configured is not None:
        root = Path(configured).expanduser()
        if not configured or not root.is_absolute() or not root.is_dir():
            raise ValueError("AP_PNC_DIR must be an existing absolute project root")
        return root.resolve()

    package_root = Path(__file__).resolve().parents[1]
    try:
        result = subprocess.run(
            ["git", "-C", str(package_root), "rev-parse", "--absolute-git-dir"],
            capture_output=True, text=True, check=True,
        )
        git_dir = Path(result.stdout.strip())
        common_file = git_dir / "commondir"
        if common_file.exists():
            return (git_dir / common_file.read_text().strip()).resolve().parent
        result = subprocess.run(
            ["git", "-C", str(package_root), "rev-parse", "--show-toplevel"],
            capture_output=True, text=True, check=True,
        )
        return Path(result.stdout.strip()).resolve()
    except (subprocess.SubprocessError, FileNotFoundError) as error:
        raise RuntimeError("Cannot locate project root; set absolute AP_PNC_DIR") from error


def project_path(path: str | Path) -> Path:
    """Resolve an absolute input or a project-root-relative input, never cwd."""
    value = Path(path).expanduser()
    if value.is_absolute():
        return value.resolve()
    if ".." in value.parts:
        raise ValueError("Project-relative paths must not contain '..'; use an absolute input")
    return (get_git_root() / value).resolve()


def artifact_path(path: str | Path) -> Path:
    """Resolve generated output and reject locations outside AP_PNC_DIR/.artifacts."""
    value = project_path(path)
    base = (get_git_root() / ".artifacts").resolve()
    if not value.is_relative_to(base):
        raise ValueError(f"Generated output must live under {base}: {value}")
    return value


def setup_env() -> None:
    """Set orchestration paths; native dependencies are configured only in Docker."""
    git_root = get_git_root()
    os.environ.setdefault("AP_PNC_DIR", str(git_root))

    # Keep every colcon run's log inside the artifacts tree (no stray ./log).
    # Overridable; explicit exports win.
    os.environ.setdefault(
        "COLCON_LOG_PATH", str(git_root / ".artifacts" / "colcon" / "log")
    )

    pythonpath = os.environ.get("PYTHONPATH", "")
    git_root_str = str(git_root)
    parts = [p for p in (git_root_str,) if p not in pythonpath]
    if parts:
        os.environ["PYTHONPATH"] = ":".join(parts) + (f":{pythonpath}" if pythonpath else "")
