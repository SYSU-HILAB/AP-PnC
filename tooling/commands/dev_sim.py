"""Isolated ROS2 development scene lifecycle."""

from pathlib import Path
from typing import Annotated

import typer

from tooling import dev_sim

app = typer.Typer(no_args_is_help=True)


@app.command()
def build():
    """Build the host-native Linux ROS2 LiDAR development image."""
    dev_sim.build()


@app.command()
def prepare(
    controller: str = typer.Option("nmpc"),
    v_max: float = typer.Option(4.0, min=0.01),
    scene: str = typer.Option("pillars", help="pillars (default) or garden-3dgs (experimental)"),
    output: Annotated[Path | None, typer.Option(help="Fresh directory under AP_PNC_DIR/.artifacts")] = None,
):
    """Stage frozen practical + lyu configs; do not start simulation."""
    typer.echo(dev_sim.prepare(controller=controller, v_max=v_max, scene=scene, output=output))


@app.command()
def run(job: Annotated[Path, typer.Argument(help="Absolute prepared job directory")]):
    """Start the isolated ROS2 scene, paused; Ctrl-C stops this container only."""
    dev_sim.run(job)
