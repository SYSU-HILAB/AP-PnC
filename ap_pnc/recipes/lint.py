import subprocess
import sys

import typer

from ap_pnc.env import setup_env

app = typer.Typer()


@app.callback(invoke_without_command=True)
def lint() -> None:
    """Run ruff linter on tests."""
    setup_env()
    subprocess.run([sys.executable, "-m", "ruff", "check", "test"], check=True)


@app.command()
def fmt() -> None:
    """Run ruff formatter on tests."""
    setup_env()
    subprocess.run([sys.executable, "-m", "ruff", "format", "test"], check=True)
