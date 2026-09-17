"""
Rich Terminal User Interface Dashboard for AP-PnC Simulation and Tracker Monitoring.

Provides an interactive multi-pane dashboard:
- Left Pane: Scenario selection, simulation controls, disturbance injection, and config writes.
- Right Pane: Live NMPC actuator outputs, odometry, tracking error, and real-time 2D trajectory & horizon canvas.
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import numpy as np
from rich.align import Align
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from ap_pnc.lu_mpc.experiments.headless_orchestrator import OrchestratorSnapshot
from ap_pnc.tui.canvas import render_trajectory_canvas
from ap_pnc.tui.orchestrator_bridge import OrchestratorBridge


def _get_key_nonblocking() -> str | None:
    """Read a single character from stdin non-blockingly on POSIX."""
    if select.select([sys.stdin], [], [], 0)[0]:
        try:
            return sys.stdin.read(1)
        except Exception:
            return None
    return None


def make_motor_bar(val: float, max_val: float = 10.0, width: int = 10) -> str:
    """Render a colored mini progress bar for motor thrust."""
    ratio = max(0.0, min(1.0, val / max_val))
    filled = int(ratio * width)
    empty = width - filled
    return f"[green]{'■' * filled}[/green][dim]{'·' * empty}[/dim] {val:4.1f}N"


def build_dashboard_layout() -> Layout:
    """Construct the master multi-pane layout."""
    layout = Layout(name="root")

    # Header and Main
    layout.split_column(
        Layout(name="header", size=3),
        Layout(name="main"),
        Layout(name="footer", size=3),
    )

    # Main split into Left (Controls) and Right (Visuals & Telemetry)
    layout["main"].split_row(
        Layout(name="left", size=42),
        Layout(name="right"),
    )

    # Left pane split
    layout["left"].split_column(
        Layout(name="controls", size=11),
        Layout(name="keybindings", size=11),
        Layout(name="writes_status"),
    )

    # Right pane split into Telemetry (top) and Canvas (bottom)
    layout["right"].split_column(
        Layout(name="telemetry", size=13),
        Layout(name="canvas"),
    )

    # Telemetry split into NMPC Output and Odometry/Tracking
    layout["telemetry"].split_row(
        Layout(name="nmpc_output"),
        Layout(name="odometry"),
    )

    return layout


def update_dashboard(
    layout: Layout,
    bridge: OrchestratorBridge,
    snap: OrchestratorSnapshot | None,
    term_width: int,
    term_height: int,
) -> None:
    """Populate layout panels from the current bridge and snapshot."""
    # ── Header ── #
    status_tag = (
        "[bold green]▶ RUNNING[/bold green]"
        if bridge.is_running
        else "[bold yellow]⏸ PAUSED[/bold yellow]"
    )
    if snap and snap.is_complete:
        status_tag = "[bold cyan]✓ COMPLETE[/bold cyan]"

    step_info = f"Step: [bold]{snap.step_idx if snap else 0}[/bold] | Time: [bold]{snap.sim_time if snap else 0.0:.2f}s[/bold]"
    header_text = Text.from_markup(
        f"[bold cyan]AP-PnC[/bold cyan] [white]| Tail-Sitter UAV NMPC Tracker & Orchestrator[/white]  "
        f"[{status_tag}]  {step_info}"
    )
    layout["header"].update(Panel(Align.center(header_text), style="cyan"))

    # ── Left: Controls Panel ── #
    ctrl_table = Table.grid(padding=(0, 1))
    ctrl_table.add_column(style="bold white")
    ctrl_table.add_column()

    ctrl_table.add_row("Scenario:", f"[bold cyan]{bridge.scenario_key}[/bold cyan]")
    ctrl_table.add_row("Target Speed:", f"{bridge.target_speed:.1f} m/s")
    ctrl_table.add_row(
        "Wind Disturbance:",
        f"Side: {bridge.wind_side:+.1f} m/s | Long: {bridge.wind_longitudinal:+.1f} m/s",
    )
    ctrl_table.add_row("Controller:", "NMPC (ACADOS SQP-RTI, N=10)")
    ctrl_table.add_row("Dynamics:", "CasADi GdModelSimulator (2.0 kg)")

    layout["controls"].update(
        Panel(ctrl_table, title="[bold]Parameters & Mode[/bold]", border_style="blue")
    )

    # ── Left: Keybindings Panel ── #
    keys_table = Table.grid(padding=(0, 1))
    keys_table.add_column(style="yellow bold")
    keys_table.add_column(style="dim white")

    keys_table.add_row("[Space]", "Play / Pause simulation")
    keys_table.add_row("[s]", "Single step simulation")
    keys_table.add_row("[r]", "Reset to initial state")
    keys_table.add_row("[1 / 2 / 3]", "Circle / Line / Lemniscate")
    keys_table.add_row("[w]", "Toggle wind disturbance")
    keys_table.add_row("[v]", "Cycle velocity (6/10/12/15)")
    keys_table.add_row("[a]", "Apply config writes (YAML)")
    keys_table.add_row("[q]", "Quit dashboard")

    layout["keybindings"].update(
        Panel(keys_table, title="[bold]Interactive Controls[/bold]", border_style="yellow")
    )

    # ── Left: Writes Status Panel ── #
    write_text = Text.from_markup(
        f"[bold]Last Action:[/bold]\n{bridge.last_write_status}\n\n"
        f"[dim]Writes sync with:\n• bringup/config/planning.yaml\n• simple_sim/config/simple_sim.yaml[/dim]"
    )
    layout["writes_status"].update(
        Panel(write_text, title="[bold]Config Sync[/bold]", border_style="green")
    )

    # ── Right Top: NMPC Output ── #
    nmpc_table = Table.grid(padding=(0, 1))
    nmpc_table.add_column(style="white", width=12)
    nmpc_table.add_column()

    if snap:
        u = snap.motor_thrusts
        nmpc_table.add_row("Motor 0 (FL):", make_motor_bar(u[0]))
        nmpc_table.add_row("Motor 1 (FR):", make_motor_bar(u[1]))
        nmpc_table.add_row("Motor 2 (RL):", make_motor_bar(u[2]))
        nmpc_table.add_row("Motor 3 (RR):", make_motor_bar(u[3]))
        nmpc_table.add_row("Total Thrust:", f"[bold green]{snap.total_thrust:.2f} N[/bold green]")
        nmpc_table.add_row("Pitch SP:", f"{snap.pitch_des:+.1f}°")
        nmpc_table.add_row(
            "Rates SP:",
            f"[{snap.rates_des[0]:+.2f}, {snap.rates_des[1]:+.2f}, {snap.rates_des[2]:+.2f}] rad/s",
        )
    else:
        nmpc_table.add_row("Status:", "Waiting for simulation step...")

    layout["nmpc_output"].update(
        Panel(nmpc_table, title="[bold]NMPC Actuator Outputs[/bold]", border_style="magenta")
    )

    # ── Right Top: Odometry & Tracking Metrics ── #
    odom_table = Table.grid(padding=(0, 1))
    odom_table.add_column(style="white", width=12)
    odom_table.add_column()

    if snap:
        p = snap.current_pos
        v = snap.current_vel
        speed = float(np.linalg.norm(v))
        rpy = snap.current_rpy
        odom_table.add_row("Position:", f"X: {p[0]:+6.2f}  Y: {p[1]:+6.2f}  Z: {p[2]:+5.2f} m")
        odom_table.add_row(
            "Velocity:", f"[{v[0]:+5.1f}, {v[1]:+5.1f}, {v[2]:+5.1f}]  |v|={speed:4.1f}m/s"
        )
        odom_table.add_row(
            "Attitude RPY:", f"R: {rpy[0]:+5.1f}°  P: {rpy[1]:+5.1f}°  Y: {rpy[2]:+5.1f}°"
        )
        odom_table.add_row(
            "Target Pos:",
            f"X: {snap.ref_pos[0]:+6.2f}  Y: {snap.ref_pos[1]:+6.2f}  Z: {snap.ref_pos[2]:+5.2f} m",
        )
        odom_table.add_row("Pos Error |e|:", f"[bold red]{snap.pos_error:6.3f} m[/bold red]")
        odom_table.add_row(
            "RMS Error:",
            f"[bold cyan]{snap.rms_error:6.3f} m[/bold cyan] (Max: {snap.max_error:5.2f}m)",
        )
    else:
        odom_table.add_row("Status:", "No odometry available")

    layout["odometry"].update(
        Panel(odom_table, title="[bold]Odometry & Tracking Error[/bold]", border_style="cyan")
    )

    # ── Right Bottom: 2D Trajectory Projection Canvas ── #
    canvas_w = max(35, term_width - 48)
    canvas_h = max(8, term_height - 24)

    cur_head = snap.current_rpy[2] * np.pi / 180.0 if snap else 0.0
    canvas_text = render_trajectory_canvas(
        ref_pts=snap.ref_trajectory if snap else None,
        pred_pts=snap.pred_trajectory if snap else None,
        cur_pos=snap.current_pos if snap else None,
        cur_heading=cur_head,
        width=canvas_w,
        height=canvas_h,
        plane="xy",
    )
    layout["canvas"].update(
        Panel(
            canvas_text,
            title="[bold]Trajectory Tracking & NMPC Horizon Preview (X-Y Projection)[/bold]",
            border_style="green",
        )
    )

    # ── Footer ── #
    footer_text = Text.from_markup(
        "[dim]Press [yellow]Space[/yellow] to Play/Pause | [yellow]1/2/3[/yellow] Switch Scenario | [yellow]a[/yellow] Apply Writes | [yellow]q[/yellow] Quit[/dim]"
    )
    layout["footer"].update(Panel(Align.center(footer_text), style="dim"))


def run_tui_dashboard(
    scenario: str = "open_circle",
    speed: float = 10.0,
    headless_steps: int | None = None,
) -> None:
    """
    Launch the interactive TUI Dashboard.

    Args:
        scenario: Initial scenario key ('open_circle', 'open_line', 'open_lemniscate')
        speed: Target velocity in m/s
        headless_steps: If provided, runs non-interactively for N steps and prints summary.
    """
    console = Console()
    bridge = OrchestratorBridge(initial_scenario=scenario)
    if speed != 10.0:
        bridge.set_speed(speed)

    # Non-interactive headless runner mode (for scripting / test verification)
    if headless_steps is not None or not sys.stdin.isatty():
        steps = headless_steps or 100
        console.print(f"[bold cyan]Running {steps} steps in headless tracker mode...[/bold cyan]")
        for _ in range(steps):
            snap = bridge.step()
            if snap.is_complete:
                break
        final_snap = (
            bridge.get_snapshot() if hasattr(bridge, "get_snapshot") else bridge.last_snapshot
        )
        assert final_snap is not None
        console.print("[bold green]✓ Headless run complete![/bold green]")
        console.print(f"  Final Position: {final_snap.current_pos}")
        console.print(f"  RMS Error: {final_snap.rms_error:.4f} m")
        console.print(f"  Max Error: {final_snap.max_error:.4f} m")
        return

    # Interactive TUI mode with termios non-blocking keyboard input
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    layout = build_dashboard_layout()

    try:
        with Live(layout, console=console, refresh_per_second=20, screen=True):
            while True:
                # 1. Handle keyboard input
                key = _get_key_nonblocking()
                if key:
                    if key == "q" or key == "\x03":  # q or Ctrl-C
                        break
                    elif key == " ":
                        bridge.is_running = not bridge.is_running
                    elif key == "s":
                        bridge.step()
                    elif key == "r":
                        bridge.reset()
                    elif key == "1":
                        bridge.switch_scenario("open_circle")
                    elif key == "2":
                        bridge.switch_scenario("open_line")
                    elif key == "3":
                        bridge.switch_scenario("open_lemniscate")
                    elif key == "w":
                        # Toggle wind: 0 -> 2 -> 4 -> 0
                        next_wind = (
                            2.0
                            if bridge.wind_side == 0.0
                            else (4.0 if bridge.wind_side == 2.0 else 0.0)
                        )
                        bridge.set_wind(wind_side=next_wind, wind_longitudinal=next_wind * 0.5)
                    elif key == "v":
                        # Cycle speed: 6 -> 10 -> 12 -> 15 -> 6
                        speeds = [6.0, 10.0, 12.0, 15.0]
                        idx = (
                            speeds.index(bridge.target_speed)
                            if bridge.target_speed in speeds
                            else 0
                        )
                        next_speed = speeds[(idx + 1) % len(speeds)]
                        bridge.set_speed(next_speed)
                    elif key == "a":
                        bridge.apply_writes_to_config()

                # 2. Advance simulation if running
                if bridge.is_running:
                    bridge.step()

                # 3. Update dashboard
                term_w, term_h = console.size
                update_dashboard(layout, bridge, bridge.last_snapshot, term_w, term_h)
                time.sleep(0.04)  # ~25Hz UI tick

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
