import subprocess
import sys

import typer

from ap_pnc.env import get_git_root, setup_env

app = typer.Typer()


@app.callback(invoke_without_command=True)
def cocp() -> None:
    """Generate acados OCP C code for the C++ NMPC solver."""
    setup_env()
    git_root = get_git_root()
    script = git_root / "src" / "c_nmpc" / "nmpc" / "create_ocp_or_sim.py"
    typer.echo(f"Running: {script}")
    subprocess.run([sys.executable, str(script)], cwd=script.parent, check=True)
