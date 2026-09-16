import os
import subprocess
import sys
import tempfile
from pathlib import Path

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()


def _build_acados_static(git_root: Path) -> None:
    """Build acados as static libs into .artifacts/acados_static."""
    src = git_root / "mpc_framework"
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


def _build_wrapper_and_bundle(git_root: Path) -> None:
    """Build libnmpc_solver.a, then merge with static acados into libnmpc_bundle.a."""
    wrapper_src = git_root / "ros_packages" / "c_nmpc" / "nmpc_wrapper"
    build_dir = git_root / ".artifacts" / "nmpc_wrapper"
    bundle = build_dir / "libnmpc_bundle.a"

    if bundle.exists():
        bundle.unlink()

    # Build wrapper .a (includes generated solver .c files)
    typer.echo("Building nmpc_wrapper...")
    subprocess.run(
        ["cmake", "-S", str(wrapper_src), "-B", str(build_dir),
         "-DAP_PNC_DIR=" + str(git_root),
         "-DACADOS_ROOT=" + str(git_root / ".artifacts" / "acados")],
        check=True,
    )
    subprocess.run(
        ["make", "-j", str(os.cpu_count() or 4), "-C", str(build_dir)],
        check=True,
    )

    # Merge: extract .o from wrapper.a + acados .a files, combine into bundle.a
    typer.echo("Merging into libnmpc_bundle.a...")
    static_dir = git_root / ".artifacts" / "acados_static"

    with tempfile.TemporaryDirectory() as tmp:
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
def cocp() -> None:
    """Generate acados OCP C code and build libnmpc_bundle.a for ROS2 linking.

    libnmpc_bundle.a is a single static archive containing the wrapper,
    generated solver, and statically-linked acados/hpipm/blasfeo.
    """
    setup_env()
    git_root = get_git_root()
    script = git_root / "ros_packages" / "c_nmpc" / "nmpc" / "create_ocp_or_sim.py"
    gen_dir = git_root / ".artifacts" / "c_generated_code"
    gen_dir.mkdir(parents=True, exist_ok=True)
    typer.echo(f"Running: {script}")
    # json_file is relative to cwd: codegen C code + JSON land in .artifacts/c_generated_code
    subprocess.run([sys.executable, str(script)], cwd=gen_dir, check=True)

    _build_acados_static(git_root)
    _build_wrapper_and_bundle(git_root)
