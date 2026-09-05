import subprocess

import typer

from ap_pnc.env import get_git_root

app = typer.Typer()


def _compose(*args: str) -> None:
    git_root = get_git_root()
    compose_file = git_root / "simulations" / "gazebo" / "docker" / "docker-compose.yml"
    cmd = ["docker", "compose", "-f", str(compose_file), *args]
    subprocess.run(cmd, check=True)


@app.command()
def build() -> None:
    """Build Docker images."""
    _compose("build")


@app.command()
def up() -> None:
    """Start Docker containers."""
    _compose("up", "-d")


@app.command()
def down() -> None:
    """Stop Docker containers."""
    _compose("down")
