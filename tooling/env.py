import os
import platform
import subprocess
import sys
from pathlib import Path

IS_LINUX = sys.platform == "linux"
IS_MACOS = sys.platform == "darwin"


def _t_renderer_url() -> str:
    """t_renderer release asset for the host.

    Only Linux builds are published (no macOS asset exists), so acados codegen
    requires a Linux development host.
    """
    if not IS_LINUX:
        raise RuntimeError(
            "t_renderer has no build for this platform; the acados codegen "
            "requires a Linux development host (macOS is not supported)"
        )
    arch = {"x86_64": "amd64", "AMD64": "amd64", "aarch64": "arm64", "arm64": "arm64"}.get(
        platform.machine()
    )
    if not arch:
        raise RuntimeError(f"Unsupported t_renderer arch: {platform.machine()}")
    return (
        "https://github.com/acados/tera_renderer/releases/download/"
        f"v0.2.0/t_renderer-v0.2.0-linux-{arch}"
    )


def ensure_t_renderer() -> Path | None:
    """Make sure t_renderer exists; acados_template would otherwise ask
    interactively whether to download it (hangs in non-tty subprocesses).

    Returns None on non-Linux hosts (acados codegen is Linux-only); callers that
    actually generate code must check for that.
    """
    git_root = get_git_root()
    bin_dir = git_root / ".artifacts" / "acados" / "bin"
    renderer = bin_dir / "t_renderer"
    if renderer.exists():
        return renderer
    if not IS_LINUX:
        return None
    bin_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(["curl", "-fL", _t_renderer_url(), "-o", str(renderer)], check=True)
    renderer.chmod(0o755)
    return renderer


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
    ACADOS_PYTHON_INTERFACE_PATH falls back to the fetched source checkout
    (.artifacts/acados_src) for the Python interface.
    """
    git_root = get_git_root()
    os.environ.setdefault("AP_PNC_DIR", str(git_root))
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(git_root / ".artifacts" / "acados"))
    os.environ.setdefault(
        "ACADOS_PYTHON_INTERFACE_PATH",
        str(
            git_root
            / ".artifacts"
            / "acados_src"
            / "interfaces"
            / "acados_template"
            / "acados_template"
        ),
    )

    lib_path = str(git_root / ".artifacts" / "acados" / "lib")
    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    if lib_path not in existing_ld:
        os.environ["LD_LIBRARY_PATH"] = f"{lib_path}:{existing_ld}"

    # Preload acados shared libraries globally so dynamically loaded solvers locate them
    lib_dir = git_root / ".artifacts" / "acados" / "lib"
    if lib_dir.exists():
        import ctypes

        for libname in ("libblasfeo.so", "libhpipm.so", "libacados.so"):
            target = lib_dir / libname
            if target.exists():
                try:
                    ctypes.CDLL(str(target), mode=ctypes.RTLD_GLOBAL)
                except OSError:
                    pass

    ensure_t_renderer()

    # Keep every colcon run's log inside the artifacts tree (no stray ./log).
    # Overridable; explicit exports win.
    os.environ.setdefault(
        "COLCON_LOG_PATH", str(git_root / ".artifacts" / "colcon" / "log")
    )

    pythonpath = os.environ.get("PYTHONPATH", "")
    git_root_str = str(git_root)
    bindings_dir = str(git_root / ".artifacts" / "bindings")
    parts = [p for p in (git_root_str, bindings_dir) if p not in pythonpath]
    if parts:
        os.environ["PYTHONPATH"] = ":".join(parts) + (f":{pythonpath}" if pythonpath else "")

    if ros:
        ros_env = source_ros2_env()
        os.environ.update(ros_env)
        # ROS sourcing must not move colcon logs back to the source tree.
        os.environ.setdefault(
            "COLCON_LOG_PATH", str(git_root / ".artifacts" / "colcon" / "log")
        )
