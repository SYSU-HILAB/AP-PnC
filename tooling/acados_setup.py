import os
import shutil
import subprocess
import sys

import typer

from tooling.env import ensure_t_renderer, get_git_root, setup_env

# Internal library, not a CLI command: `ap-pnc gen-nmpc-lib` calls
# install_acados_stack() on first use, so a separate install step would only be
# a second way to do the same thing.
#
# acados is fetched on demand into .artifacts/acados_src and locked to a pinned
# commit (no submodule, nothing tracked at the repo top level).
ACADOS_REPO = "https://github.com/acados/acados.git"
ACADOS_COMMIT = "2b28dc320a7d17d9f9b6ffefb59ee022d63bab83"  # v0.5.3-35


def _fetch_acados_source() -> None:
    """Clone or sync the locked acados source into .artifacts/acados_src."""
    git_root = get_git_root()
    acados_src = git_root / ".artifacts" / "acados_src"
    def git(*args: str) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["git", "-C", str(acados_src), *args], capture_output=True, text=True, check=True
        )

    if not (acados_src / ".git").exists():
        typer.echo(f"Cloning acados at {ACADOS_COMMIT[:10]}...")
        subprocess.run(
            ["git", "-C", str(git_root), "clone", ACADOS_REPO, str(acados_src)], check=True
        )
        git("checkout", ACADOS_COMMIT)
        git("submodule", "update", "--init", "--recursive")
        return
    head = git("rev-parse", "HEAD").stdout.strip()
    if head != ACADOS_COMMIT:
        typer.echo(f"Syncing acados {head[:10]} -> {ACADOS_COMMIT[:10]}...")
        git("fetch", "origin")
        git("checkout", ACADOS_COMMIT)
        git("submodule", "update", "--init", "--recursive")


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
        ["cmake", "-S", str(acados_src), "-B", str(build_dir),
         "-DCMAKE_INSTALL_PREFIX=" + str(install_prefix)],
        cwd=build_dir,
        check=True,
    )
    subprocess.run(["cmake", "--build", str(build_dir), "-j4"], check=True)
    subprocess.run(["cmake", "--install", str(build_dir)], check=True)

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


def install_acados_stack() -> None:
    """Install the acados toolchain: venv, C libs (shared + static), t_renderer, acados_template."""
    git_root = get_git_root()
    subprocess.run(["uv", "sync", "--project", str(git_root), "--group", "dev"], cwd=git_root, check=True)
    _fetch_acados_source()
    setup_env()
    _build_acados_c()
    _build_acados_static()
    _download_t_renderer()
    _install_acados_template()
    typer.echo("acados toolchain ready.")
