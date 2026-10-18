"""Host-side orchestration for Linux build/test tools; no native execution path."""

import shutil
import subprocess
from pathlib import Path

from tooling.docker_runtime import ensure_bundle, target_arch
from tooling.env import get_git_root

CONTAINER_ROOT = Path("/workspace")


def run(
    command: list[str],
    *,
    arch: str = "native",
    clean: bool = False,
    solver: bool = False,
) -> None:
    """Run build/test tools with per-target Linux artifacts and read-only sources."""
    root = get_git_root()
    target = target_arch(arch)
    platform = f"linux/{target}"
    if solver:
        ensure_bundle(root, target)
    cache = root / ".artifacts/docker-tools" / target
    if clean:
        shutil.rmtree(cache, ignore_errors=True)
    cache.mkdir(parents=True, exist_ok=True)
    bundles = root / ".artifacts/nmpc_solver"
    bundles.mkdir(parents=True, exist_ok=True)
    image = f"ap-pnc-tools:{target}"
    subprocess.run(
        [
            "docker", "build", "--platform", platform, "--target", "tools",
            "-f", str(root / "core/docker/ros2-base.dockerfile"),
            "-t", image, str(root),
        ],
        check=True,
    )
    subprocess.run(
        [
            "docker", "run", "--rm", "--platform", platform, "--ipc=host",
            "--network", "none",
            "--entrypoint", "/bin/bash", "--workdir", str(CONTAINER_ROOT),
            "-e", f"AP_PNC_DIR={CONTAINER_ROOT}",
            "-e", "PYTHONPATH=/workspace:/workspace/.artifacts/bindings",
            "-e", "PYTHONDONTWRITEBYTECODE=1",
            "-v", f"{root}:{CONTAINER_ROOT}:ro",
            "-v", f"{cache}:{CONTAINER_ROOT}/.artifacts:rw",
            "-v", f"{bundles}:{CONTAINER_ROOT}/.artifacts/nmpc_solver:ro",
            image, "-c",
            'set -e; source /opt/ros/humble/setup.bash; '
            'setup="$AP_PNC_DIR/.artifacts/colcon/install/setup.bash"; '
            'if [[ -f "$setup" ]]; then source "$setup"; fi; exec "$@"',
            "tools", *command,
        ],
        check=True,
    )
