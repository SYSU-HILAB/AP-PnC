import os
import subprocess
import sys

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()


def _ensure_submodule() -> None:
    git_root = get_git_root()
    acados_dir = git_root / "mpc_framework"
    if not (acados_dir / "CMakeLists.txt").exists():
        subprocess.run(
            ["git", "submodule", "update", "--init", "--recursive", "mpc_framework"],
            cwd=git_root,
            check=True,
        )


def _build_acados_c() -> None:
    git_root = get_git_root()
    lib_acados = git_root / "mpc_framework" / "lib" / "libacados.so"
    if lib_acados.exists():
        return
    acados_src = git_root / "mpc_framework"
    build_dir = git_root / "mpc_framework" / "build"
    build_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["cmake", str(acados_src)],
        cwd=build_dir,
        check=True,
    )
    subprocess.run(["make", "-j4"], cwd=build_dir, check=True)
    subprocess.run(["make", "install"], cwd=build_dir, check=True)


def _download_t_renderer() -> None:
    git_root = get_git_root()
    bin_dir = git_root / "mpc_framework" / "bin"
    renderer = bin_dir / "t_renderer"
    if renderer.exists():
        return
    bin_dir.mkdir(parents=True, exist_ok=True)
    url = "https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64"
    subprocess.run(["curl", "-L", url, "-o", str(renderer)], check=True)
    renderer.chmod(0o755)


def _build_acados_static() -> None:
    git_root = get_git_root()
    build_dir = git_root / "build" / "acados_static"
    src = git_root / "mpc_framework"
    if (build_dir / "acados" / "libacados.a").exists():
        return
    build_dir.mkdir(parents=True, exist_ok=True)
    nproc = os.cpu_count() or 4
    subprocess.run(
        ["cmake", "-S", str(src), "-B", str(build_dir),
         "-DBUILD_SHARED_LIBS=OFF"],
        check=True,
    )
    subprocess.run(["make", "-j", str(nproc), "-C", str(build_dir)], check=True)


def _install_acados_template() -> None:
    result = subprocess.run(
        [sys.executable, "-m", "pip", "show", "acados-template"],
        capture_output=True,
    )
    if result.returncode != 0:
        git_root = get_git_root()
        subprocess.run(
            [
                "uv", "pip", "install", "-e",
                str(git_root / "mpc_framework" / "interfaces" / "acados_template"),
            ],
            check=True,
        )


@app.callback(invoke_without_command=True)
def install() -> None:
    """Install everything: venv, acados C libs (shared + static), t_renderer, acados_template."""
    git_root = get_git_root()
    subprocess.run(["uv", "sync", "--group", "dev"], cwd=git_root, check=True)
    _ensure_submodule()
    setup_env()
    _build_acados_c()
    _build_acados_static()
    _download_t_renderer()
    _install_acados_template()
    typer.echo("Installation complete.")
