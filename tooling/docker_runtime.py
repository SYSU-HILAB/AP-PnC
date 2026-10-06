"""Host-native Docker targeting; project resources never depend on cwd."""

import os
import platform
import subprocess
from collections.abc import Sequence
from pathlib import Path

import typer

ARCH_ALIASES = {
    "arm64": "arm64",
    "aarch64": "arm64",
    "amd64": "amd64",
    "x86_64": "amd64",
}
SOLVER_ARCH = {"arm64": "aarch64", "amd64": "x86_64"}


def target_arch(value: str = "native") -> str:
    machine = platform.machine().lower() if value == "native" else value.lower()
    try:
        return ARCH_ALIASES[machine]
    except KeyError as error:
        raise typer.BadParameter("--arch must be native, arm64/aarch64 or amd64/x86_64") from error


def compose_env(root: Path, arch: str = "native", nav_target: str = "sim") -> dict[str, str]:
    if not root.is_absolute():
        raise ValueError("Project root must be absolute")
    if nav_target not in ("sim", "real"):
        raise typer.BadParameter("--nav-target must be sim or real")
    return {
        **os.environ,
        "AP_PNC_DIR": str(root),
        "AP_PNC_ARCH": target_arch(arch),
        "AP_PNC_NAV_TARGET": nav_target,
    }


def compose(
    root: Path,
    compose_file: Path,
    args: Sequence[str],
    *,
    arch: str = "native",
    gui: bool = False,
    nav_target: str = "sim",
) -> None:
    if not compose_file.is_absolute():
        raise ValueError("Compose file must be absolute")
    command = ["docker", "compose", "-f", str(compose_file)]
    if args and args[0] == "build":
        command += ["--profile", "build"]
    if "--progress=plain" in args:
        command += ["--progress", "plain"]
        args = [arg for arg in args if arg != "--progress=plain"]
    if gui or (args and args[0] == "down"):
        command += ["--profile", "gui"]
    subprocess.run(
        [*command, *args],
        env=compose_env(root, arch, nav_target),
        cwd=root,
        check=True,
    )


def ensure_bundle(root: Path, arch: str = "native") -> None:
    """Docker builds consume only the selected arch-tagged archive, never the host alias."""
    if not root.is_absolute():
        raise ValueError("Project root must be absolute")
    solver_arch = SOLVER_ARCH[target_arch(arch)]
    bundle = root / ".artifacts/nmpc_solver" / f"libnmpc_bundle_{solver_arch}.a"
    if not bundle.is_file():
        from tooling.commands.gen_nmpc_lib import cocp

        cocp(arch=solver_arch)
    if not bundle.is_file():
        raise RuntimeError(f"Solver builder did not produce {bundle}")
