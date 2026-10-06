import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import typer

from tooling.env import IS_LINUX, get_git_root, setup_env

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
        "-e", "NMPC_BUNDLE_ALIAS=1",
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
    if target_arch == _host_arch():
        _publish_bundle(tagged, solver_dir / "libnmpc_bundle.a")
    typer.echo(f"Bundle: {tagged}")


def _build_acados_static(git_root: Path) -> None:
    """Build acados as static libs into .artifacts/acados_static."""
    src = git_root / ".artifacts" / "acados_src"
    build_dir = git_root / ".artifacts" / "acados_static"
    if not (build_dir / "acados" / "libacados.a").exists():
        typer.echo("Building acados static libs...")
        subprocess.run(
            ["cmake", "-S", str(src), "-B", str(build_dir),
             "-DBUILD_SHARED_LIBS=OFF", "-DCMAKE_BUILD_TYPE=Release"],
            check=True,
        )
        subprocess.run(
            ["make", "-j", str(os.cpu_count() or 4), "-C", str(build_dir)],
            check=True,
        )
    else:
        typer.echo("acados static libs already built, skipping.")


def _build_wrapper_and_bundle(
    git_root: Path,
    staging_root: Path | None = None,
    static_dir: Path | None = None,
) -> None:
    """Build libnmpc_solver.a, then merge with static acados into libnmpc_bundle.a.

    The staging locations are arguments: the Docker builder image points them
    at /opt, a host run takes the .artifacts defaults.
    """
    wrapper_src = git_root / "core" / "ros_packages" / "nmpc" / "solver"
    build_dir = git_root / ".artifacts" / "nmpc_solver"
    # Arguments, not variables: the builder image passes /opt paths, a host run
    # takes the .artifacts defaults. AP_PNC_DIR stays the only variable.
    staging_root = staging_root or (git_root / ".artifacts" / "acados")
    static_dir = static_dir or (git_root / ".artifacts" / "acados_static")
    bundle = build_dir / "libnmpc_bundle.a"
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

    typer.echo("Merging into libnmpc_bundle.a...")

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


def _ensure_prerequisites() -> None:
    """Run install automatically when acados prerequisites are missing."""
    git_root = get_git_root()
    staged_lib = git_root / ".artifacts" / "acados" / "lib" / "libacados.so"
    renderer = git_root / ".artifacts" / "acados" / "bin" / "t_renderer"
    source = git_root / ".artifacts" / "acados_src" / "CMakeLists.txt"
    try:
        import acados_template  # noqa: F401
        have_template = True
    except ImportError:
        have_template = False

    if staged_lib.exists() and renderer.exists() and source.exists() and have_template:
        return

    typer.echo("acados prerequisites missing, running install...")
    from tooling import acados_setup

    acados_setup.install_acados_stack()


@app.callback(invoke_without_command=True)
def cocp(
    arch: str = typer.Option(
        "native",
        "--arch",
        help="native | aarch64 | x86_64. Native uses the local toolchain "
             "(Linux only); everything else builds in the Docker builder image.",
    ),
) -> None:
    """Generate acados OCP C code and build libnmpc_bundle.a for ROS2 linking.

    libnmpc_bundle.a is a single static archive containing the wrapper,
    generated solver, and statically-linked acados/hpipm/blasfeo. Output is
    arch-tagged: .artifacts/nmpc_solver/libnmpc_bundle_<arch>.a (the
    unsuffixed libnmpc_bundle.a stays as the host-native alias).
    Prerequisites for the native path (acados source, staged libs,
    t_renderer, acados_template) are installed automatically on first use;
    the Docker path (any non-Linux host or explicit --arch) is self-contained.
    """
    target = _host_arch() if arch == "native" else arch
    if target not in ("aarch64", "x86_64"):
        raise typer.BadParameter(
            f"unsupported arch '{arch}' (expected native | aarch64 | x86_64)"
        )

    git_root = get_git_root()
    use_docker = arch != "native" or not IS_LINUX
    if use_docker:
        _docker_build_bundle(git_root, target)
        return

    _ensure_prerequisites()
    setup_env()
    script = git_root / "tooling" / "nmpc_gen" / "create_ocp.py"
    gen_dir = git_root / ".artifacts" / "c_generated_code"
    gen_dir.mkdir(parents=True, exist_ok=True)
    typer.echo(f"Running: {script}")
    # The generator explicitly resolves code and JSON outputs against AP_PNC_DIR.
    subprocess.run([sys.executable, str(script)], cwd=gen_dir, check=True)

    _build_acados_static(git_root)
    _build_wrapper_and_bundle(git_root)

    bundle = git_root / ".artifacts" / "nmpc_solver" / "libnmpc_bundle.a"
    tagged = bundle.with_name(f"libnmpc_bundle_{target}.a")
    shutil.copy2(bundle, tagged)
    typer.echo(f"Arch-tagged bundle: {tagged}")
