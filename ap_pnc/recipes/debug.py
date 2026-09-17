import subprocess

import typer

from ap_pnc.env import get_git_root

app = typer.Typer(help="Interactive debugging helpers.")


@app.command()
def pcd(
    file: str = typer.Argument(..., help="PCD file dumped by simple_sim"),
    info: bool = typer.Option(False, "--info", help="Print stats instead of opening a window"),
) -> None:
    """Visualize a simple_sim ASCII PCD with open3d (isolated PEP 723 deps)."""
    script = get_git_root() / "ap_pnc" / "tools" / "viz_pcd.py"
    args = ["uv", "run", "--script", str(script), file]
    if info:
        args.append("--info")
    subprocess.run(args, check=True)
