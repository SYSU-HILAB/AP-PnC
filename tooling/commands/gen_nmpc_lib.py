import os
import platform
import shutil
import subprocess
import tempfile
from pathlib import Path

import typer

from tooling.env import get_git_root

app = typer.Typer()


def _host_arch() -> str:
    """Normalize the host machine to an ELF arch name (aarch64 | x86_64)."""
    machine = platform.machine().lower()
    return {
        "arm64": "aarch64",
        "aarch64": "aarch64",
        "amd64": "x86_64",
        "x86_64": "x86_64",
        "x86": "x86_64",
    }.get(machine, machine)


def _publish_bundle(source: Path, destination: Path) -> None:
    """Atomic replacement keeps concurrent image builds from reading partial archives."""
    with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=".a", delete=False) as file:
        temporary = Path(file.name)
    try:
        shutil.copy2(source, temporary)
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)


def _docker_build_bundle(git_root: Path, target_arch: str) -> None:
    """Build the bundle inside the pinned Docker builder image.

    The image bakes the whole toolchain (acados commit, t_renderer,
    acados_template), so the host needs nothing but Docker. Produces
    .artifacts/nmpc_solver/libnmpc_bundle_<arch>.a.
    """
    if shutil.which("docker") is None:
        raise typer.BadParameter(
            "docker is required to build the bundle for this arch on this host"
        )
    from tooling.acados_setup import ACADOS_COMMIT  # single pin source

    docker_platform = {"aarch64": "linux/arm64", "x86_64": "linux/amd64"}[
        target_arch
    ]
    tag = f"ap-pnc-nmpc-builder:{ACADOS_COMMIT[:10]}-{target_arch}"
    typer.echo(
        f"Building {tag} (toolchain pinned at acados {ACADOS_COMMIT[:10]})..."
    )
    subprocess.run(
        [
            "docker", "build", "--platform", docker_platform,
            "-f", str(git_root / "core" / "docker" / "nmpc-builder.dockerfile"),
            "--build-arg", f"ACADOS_COMMIT={ACADOS_COMMIT}",
            "-t", tag,
            str(git_root),
        ],
        check=True,
    )
    # Codegen and CMake caches are target-specific. Never let a Docker build
    # overwrite host-native (possibly macOS) or another Linux arch's artifacts.
    artifacts = git_root / ".artifacts"
    workspace = artifacts / "nmpc_build" / target_arch
    workspace.mkdir(parents=True, exist_ok=True)
    run_cmd = [
        "docker", "run", "--rm", "--platform", docker_platform,
        "--ipc=host",
        "-v", f"{git_root}:/ws:ro",
        "-v", f"{workspace}:/ws/.artifacts:rw",
        "-e", "AP_PNC_DIR=/ws",
    ]
    env = {**os.environ, "AP_PNC_DIR": str(git_root)}
    subprocess.run(run_cmd + [tag], check=True, env=env)

    name = f"libnmpc_bundle_{target_arch}.a"
    built = workspace / "nmpc_solver" / name
    if not built.is_file():
        raise RuntimeError(f"builder container finished but {built} is missing")
    solver_dir = artifacts / "nmpc_solver"
    solver_dir.mkdir(parents=True, exist_ok=True)
    tagged = solver_dir / name
    _publish_bundle(built, tagged)
    typer.echo(f"Bundle: {tagged}")


def _build_wrapper_and_bundle(
    git_root: Path,
    *,
    arch: str,
    staging_root: Path,
    static_dir: Path,
) -> None:
    """Container-only wrapper build using the image's explicit toolchain paths."""
    wrapper_src = git_root / "core" / "ros_packages" / "nmpc" / "src"
    build_dir = git_root / ".artifacts" / "nmpc_solver"
    if arch not in ("aarch64", "x86_64"):
        raise ValueError(f"unsupported solver architecture: {arch}")
    bundle = build_dir / f"libnmpc_bundle_{arch}.a"
    build_dir.mkdir(parents=True, exist_ok=True)

    # A cache from a different checkout path (e.g. a container build at
    # /workspace vs this run at /ws) poisons cmake; drop it on mismatch.
    cache_file = build_dir / "CMakeCache.txt"
    if cache_file.exists():
        cached_home = ""
        for line in cache_file.read_text(errors="ignore").splitlines():
            if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL="):
                cached_home = line.split("=", 1)[1]
                break
        if cached_home != str(wrapper_src):
            typer.echo(
                f"Removing stale CMake cache (was configured for {cached_home})..."
            )
            shutil.rmtree(build_dir / "CMakeFiles", ignore_errors=True)
            cache_file.unlink(missing_ok=True)

    if bundle.exists():
        bundle.unlink()

    # Build wrapper .a (includes generated solver .c files)
    typer.echo("Building nmpc solver wrapper...")
    subprocess.run(
        ["cmake", "-S", str(wrapper_src), "-B", str(build_dir),
         "-DAP_PNC_DIR=" + str(git_root),
         "-DACADOS_ROOT=" + str(staging_root)],
        check=True,
    )
    subprocess.run(
        ["make", "-j", str(os.cpu_count() or 4), "-C", str(build_dir)],
        check=True,
    )

    typer.echo(f"Merging into {bundle.name}...")

    with tempfile.TemporaryDirectory(dir=build_dir, prefix="bundle_merge_") as tmp:
        tmpdir = Path(tmp)
        subprocess.run(["ar", "x", str(build_dir / "libnmpc_solver.a")],
                       cwd=tmpdir, check=True)

        # acados first so its timing.c.o is preserved (hpipm also has one)
        subprocess.run(
            ["ar", "x", str(static_dir / "acados" / "libacados.a")],
            cwd=tmpdir, check=True)
        for a_file in [
            static_dir / "external" / "hpipm" / "libhpipm.a",
            static_dir / "external" / "blasfeo" / "libblasfeo.a",
        ]:
            subprocess.run(["ar", "x", str(a_file)], cwd=tmpdir, check=True)
        # re-extract acados timing to fix hpipm overwrite
        timing_data = subprocess.check_output(
            ["ar", "p", str(static_dir / "acados" / "libacados.a"),
             "timing.c.o"])
        (tmpdir / "acados_timing.c.o").write_bytes(timing_data)

        subprocess.run(
            ["ar", "rcs", str(bundle)] + sorted(tmpdir.glob("*.o")),
            check=True,
        )

    size = bundle.stat().st_size
    typer.echo(f"Bundle: {bundle} ({size // 1024} KB)")


@app.callback(invoke_without_command=True)
def cocp(
    arch: str = typer.Option(
        "native",
        "--arch",
        help="native | aarch64 | x86_64. Native selects the host CPU architecture; "
             "all targets build in the pinned Linux Docker builder image.",
    ),
) -> None:
    """Generate an architecture-tagged Linux static NMPC bundle in Docker.

    .artifacts/nmpc_solver/libnmpc_bundle_<arch>.a contains the wrapper,
    generated solver and statically-linked acados/hpipm/blasfeo.
    There is no host-native build or untagged output alias.
    All targets use the self-contained Docker builder; no host ROS, acados
    installation or native compiler is needed.
    """
    target = _host_arch() if arch == "native" else arch
    if target not in ("aarch64", "x86_64"):
        raise typer.BadParameter(
            f"unsupported arch '{arch}' (expected native | aarch64 | x86_64)"
        )

    git_root = get_git_root()
    # One path everywhere: the pinned builder image carries acados, t_renderer and
    # acados_template, so a host needs Docker and nothing else and Linux results
    # come from the same builder as macOS.
    _docker_build_bundle(git_root, target)
