import os
import shutil
import subprocess
import sys

import typer

from ap_pnc.env import ensure_t_renderer, get_git_root, setup_env

app = typer.Typer()

# acados is fetched on demand into .artifacts/acados_src and locked to a pinned
# commit (no submodule, nothing tracked at the repo top level).
ACADOS_REPO = "https://github.com/acados/acados.git"
ACADOS_COMMIT = "2b28dc320a7d17d9f9b6ffefb59ee022d63bab83"  # v0.5.3-35


def _fetch_acados_source() -> None:
    """Clone or sync the locked acados source into .artifacts/acados_src."""
    git_root = get_git_root()
    acados_src = git_root / ".artifacts" / "acados_src"
    if not (acados_src / ".git").exists():
        typer.echo(f"Cloning acados at {ACADOS_COMMIT[:10]}...")
        subprocess.run(
            ["git", "clone", ACADOS_REPO, str(acados_src)],
            cwd=git_root,
            check=True,
        )
        subprocess.run(
            ["git", "checkout", ACADOS_COMMIT],
            cwd=acados_src,
            check=True,
        )
        subprocess.run(
            ["git", "submodule", "update", "--init", "--recursive"],
            cwd=acados_src,
            check=True,
        )
        return
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=acados_src,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    if head != ACADOS_COMMIT:
        typer.echo(f"Syncing acados {head[:10]} -> {ACADOS_COMMIT[:10]}...")
        subprocess.run(["git", "fetch", "origin"], cwd=acados_src, check=True)
        subprocess.run(
            ["git", "checkout", ACADOS_COMMIT], cwd=acados_src, check=True
        )
        subprocess.run(
            ["git", "submodule", "update", "--init", "--recursive"],
            cwd=acados_src,
            check=True,
        )


def _build_acados_c() -> None:
    """Build acados shared libs out-of-tree and stage into .artifacts/acados."""
    git_root = get_git_root()
    install_prefix = git_root / ".artifacts" / "acados"
    lib_acados = install_prefix / "lib" / "libacados.so"
    if lib_acados.exists():
        return
    acados_src = git_root / ".artifacts" / "acados_src"
    build_dir = install_prefix / "build"
    build_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["cmake", str(acados_src),
         "-DCMAKE_INSTALL_PREFIX=" + str(install_prefix)],
        cwd=build_dir,
        check=True,
    )
    subprocess.run(["make", "-j4"], cwd=build_dir, check=True)
    subprocess.run(["make", "install"], cwd=build_dir, check=True)

    # acados CMake writes these metadata files into the source tree only;
    # acados_template reads them from ACADOS_SOURCE_DIR/lib, so stage copies.
    staged_lib = install_prefix / "lib"
    for meta in ("link_libs.json", "git_commit_hash"):
        src_meta = acados_src / "lib" / meta
        if src_meta.exists() and not (staged_lib / meta).exists():
            shutil.copy2(src_meta, staged_lib / meta)


def _download_t_renderer() -> None:
    ensure_t_renderer()


def _build_acados_static() -> None:
    git_root = get_git_root()
    build_dir = git_root / ".artifacts" / "acados_static"
    src = git_root / ".artifacts" / "acados_src"
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
                str(git_root / ".artifacts" / "acados_src" / "interfaces" / "acados_template"),
            ],
            check=True,
        )


@app.callback(invoke_without_command=True)
def install() -> None:
    """Install everything: venv, acados C libs (shared + static), t_renderer, acados_template."""
    git_root = get_git_root()
    subprocess.run(["uv", "sync", "--group", "dev"], cwd=git_root, check=True)
    _fetch_acados_source()
    setup_env()
    _build_acados_c()
    _build_acados_static()
    _download_t_renderer()
    _install_acados_template()
    typer.echo("Installation complete.")
