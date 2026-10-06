import subprocess
import sys

import typer

from tooling.env import get_git_root, setup_env

app = typer.Typer()


@app.callback(invoke_without_command=True)
def lint() -> None:
    """Run ruff linter on tests."""
    setup_env()
    root = get_git_root()
    subprocess.run(
        [sys.executable, "-m", "ruff", "check", str(root / "test"),
         "--cache-dir", str(root / ".artifacts" / "ruff_cache")],
        cwd=root, check=True,
    )


@app.command()
def fmt() -> None:
    """Run ruff formatter on tests."""
    setup_env()
    root = get_git_root()
    subprocess.run(
        [sys.executable, "-m", "ruff", "format", str(root / "test"),
         "--cache-dir", str(root / ".artifacts" / "ruff_cache")],
        cwd=root, check=True,
    )
