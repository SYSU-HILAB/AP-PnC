"""
Tracker performance evaluation and interactive TUI orchestrator commands.
"""

from __future__ import annotations

import typer
from rich.console import Console

from ap_pnc.env import setup_env
from ap_pnc.lu_mpc.experiments.open_scenarios import SCENARIOS, ensure_open_scenario
from ap_pnc.tui.dashboard import run_tui_dashboard
from ap_pnc.tui.orchestrator_bridge import OrchestratorBridge

app = typer.Typer(help="Tracker evaluation, open collision-free scenarios, and interactive TUI.")
console = Console()


@app.command(name="tui")
def tui_cmd(
    scenario: str = typer.Option(
        "open_circle",
        "--scenario",
        "-s",
        help=f"Initial scenario: {list(SCENARIOS.keys())}",
    ),
    speed: float = typer.Option(10.0, "--speed", "-v", help="Target velocity [m/s]"),
) -> None:
    """Launch the interactive terminal dashboard for tracker monitoring and orchestration."""
    setup_env()
    if scenario not in SCENARIOS:
        console.print(
            f"[yellow]Unknown scenario '{scenario}'. Defaulting to 'open_circle'.[/yellow]"
        )
        scenario = "open_circle"
    run_tui_dashboard(scenario=scenario, speed=speed)


@app.command(name="headless")
def headless_cmd(
    scenario: str = typer.Option(
        "open_circle",
        "--scenario",
        "-s",
        help=f"Open scenario to run: {list(SCENARIOS.keys())}",
    ),
    speed: float = typer.Option(10.0, "--speed", "-v", help="Target velocity [m/s]"),
    steps: int = typer.Option(150, "--steps", "-n", help="Number of simulation steps to run"),
    wind_side: float = typer.Option(0.0, "--wind-side", help="Side wind disturbance [m/s]"),
    wind_long: float = typer.Option(0.0, "--wind-long", help="Longitudinal wind disturbance [m/s]"),
) -> None:
    """Run a headless tracker test on an open collision-free scenario."""
    setup_env()
    if scenario not in SCENARIOS:
        scenario = "open_circle"

    console.print(
        f"[bold cyan]Running headless tracker test on {scenario} (v={speed:.1f} m/s, steps={steps})...[/bold cyan]"
    )
    bridge = OrchestratorBridge(initial_scenario=scenario)
    bridge.set_speed(speed)
    if wind_side != 0.0 or wind_long != 0.0:
        bridge.set_wind(wind_side, wind_long)

    for i in range(steps):
        snap = bridge.step()
        if snap.is_complete:
            console.print(f"[green]Trajectory completed at step {i + 1}[/green]")
            break

    snap = bridge.last_snapshot
    assert snap is not None
    console.print("\n[bold green]✓ Tracker Performance Summary:[/bold green]")
    console.print(f"  • Steps: {snap.step_idx}")
    console.print(f"  • Sim Time: {snap.sim_time:.2f} s")
    console.print(
        f"  • Current Pos: [{snap.current_pos[0]:.2f}, {snap.current_pos[1]:.2f}, {snap.current_pos[2]:.2f}] m"
    )
    console.print(f"  • Final Error: {snap.pos_error:.4f} m")
    console.print(f"  • RMS Error:   [bold cyan]{snap.rms_error:.4f} m[/bold cyan]")
    console.print(f"  • Max Error:   {snap.max_error:.4f} m")
    console.print(f"  • Total Thrust:{snap.total_thrust:.2f} N")


@app.command(name="scenario")
def scenario_cmd(
    scenario: str = typer.Option(
        "open_circle",
        "--name",
        "-n",
        help=f"Scenario name: {list(SCENARIOS.keys())}",
    ),
    speed: float = typer.Option(10.0, "--speed", "-v", help="Target velocity [m/s]"),
    write: bool = typer.Option(
        False,
        "--write",
        "-w",
        help="Write open scenario settings to planning.yaml and simple_sim.yaml",
    ),
) -> None:
    """Generate open collision-free scenario trajectory and optionally sync YAML configs."""
    setup_env()
    if scenario not in SCENARIOS:
        console.print(
            f"[red]Error: Unknown scenario '{scenario}'. Available: {list(SCENARIOS.keys())}[/red]"
        )
        raise typer.Exit(1)

    csv_path = ensure_open_scenario(scenario, speed=speed)
    console.print(f"[green]✓ Open scenario trajectory ready at:[/green] {csv_path}")

    if write:
        bridge = OrchestratorBridge(initial_scenario=scenario)
        bridge.set_speed(speed)
        result = bridge.apply_writes_to_config()
        console.print(f"[bold green]✓ Configuration written:[/bold green] {result}")


@app.command(name="web")
def web_cmd(
    scenario: str = typer.Option(
        "open_circle",
        "--scenario",
        "-s",
        help=f"Initial scenario: {list(SCENARIOS.keys())}",
    ),
    speed: float = typer.Option(10.0, "--speed", "-v", help="Target velocity [m/s]"),
    host: str = typer.Option("127.0.0.1", "--host", "-h", help="Host interface to bind"),
    port: int = typer.Option(8080, "--port", "-p", help="Port to listen on"),
    no_browser: bool = typer.Option(False, "--no-browser", help="Don't open browser automatically"),
) -> None:
    """Launch the interactive 3D Web simulation dashboard (FastAPI + Three.js)."""
    from ap_pnc.web.server import run_web_server

    setup_env()
    if scenario not in SCENARIOS:
        scenario = "open_circle"

    bridge = OrchestratorBridge(initial_scenario=scenario)
    bridge.set_speed(speed)
    run_web_server(bridge, host=host, port=port, open_browser=not no_browser)
