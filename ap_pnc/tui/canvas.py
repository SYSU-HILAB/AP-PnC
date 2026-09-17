"""
Terminal 2D Trajectory Projection Canvas using Rich.

Renders real-time 2D projections (X-Y horizontal or X-Z vertical)
comparing tracked reference trajectory, NMPC predicted horizon,
and current drone position and heading.
"""

from __future__ import annotations

import numpy as np
from rich.text import Text


def heading_to_arrow(angle_rad: float) -> str:
    """Map yaw/heading angle in radians to a cardinal directional arrow."""
    deg = (angle_rad * 180.0 / np.pi) % 360.0
    if 45.0 <= deg < 135.0:
        return "▲"
    elif 135.0 <= deg < 225.0:
        return "◀"
    elif 225.0 <= deg < 315.0:
        return "▼"
    else:
        return "▶"


def render_trajectory_canvas(
    ref_pts: np.ndarray | None = None,
    pred_pts: np.ndarray | None = None,
    cur_pos: np.ndarray | None = None,
    cur_heading: float = 0.0,
    width: int = 56,
    height: int = 15,
    plane: str = "xy",
) -> Text:
    """
    Render a 2D projection of trajectories into a styled Rich Text canvas.

    Args:
        ref_pts: Array of reference points (M, 2) or (M, 3)
        pred_pts: Array of predicted points (N+1, 2) or (N+1, 3)
        cur_pos: Current position (2,) or (3,)
        cur_heading: Current heading in radians
        width: Canvas character width
        height: Canvas character height
        plane: Projection plane ('xy' or 'xz')

    Returns:
        Rich Text object suitable for rendering inside a Rich Panel.
    """
    idx_x = 0
    idx_y = 1 if plane == "xy" else 2

    # Collect coordinates to determine bounding box
    all_x: list[float] = []
    all_y: list[float] = []

    if ref_pts is not None and len(ref_pts) > 0:
        all_x.extend(ref_pts[:, idx_x])
        all_y.extend(ref_pts[:, idx_y])
    if pred_pts is not None and len(pred_pts) > 0:
        all_x.extend(pred_pts[:, idx_x])
        all_y.extend(pred_pts[:, idx_y])
    if cur_pos is not None:
        all_x.append(cur_pos[idx_x])
        all_y.append(cur_pos[idx_y])

    if not all_x:
        min_x, max_x = -10.0, 10.0
        min_y, max_y = -10.0, 10.0
    else:
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        # Add margin
        span_x = max(max_x - min_x, 2.0)
        span_y = max(max_y - min_y, 2.0)
        # Maintain aspect ratio loosely
        pad_x = span_x * 0.08
        pad_y = span_y * 0.08
        min_x -= pad_x
        max_x += pad_x
        min_y -= pad_y
        max_y += pad_y

    # Grid initialization: stores (char, style)
    grid = [[(" ", "dim") for _ in range(width)] for _ in range(height)]

    def to_grid(x: float, y: float) -> tuple[int, int]:
        c = int((x - min_x) / (max_x - min_x) * (width - 1))
        r = height - 1 - int((y - min_y) / (max_y - min_y) * (height - 1))
        return max(0, min(height - 1, r)), max(0, min(width - 1, c))

    # 1. Plot reference trajectory (cyan dots)
    if ref_pts is not None and len(ref_pts) > 0:
        step = max(1, len(ref_pts) // (width * 2))
        for p in ref_pts[::step]:
            r, c = to_grid(p[idx_x], p[idx_y])
            grid[r][c] = ("·", "cyan dim")

    # 2. Plot predicted horizon (bright green asterisks / lines)
    if pred_pts is not None and len(pred_pts) > 0:
        for i, p in enumerate(pred_pts):
            r, c = to_grid(p[idx_x], p[idx_y])
            style = "bold green" if i == len(pred_pts) - 1 else "green"
            char = "★" if i == len(pred_pts) - 1 else "*"
            grid[r][c] = (char, style)

    # 3. Plot current drone position (bold yellow/red arrow)
    if cur_pos is not None:
        r, c = to_grid(cur_pos[idx_x], cur_pos[idx_y])
        arrow = heading_to_arrow(cur_heading) if plane == "xy" else "▲"
        grid[r][c] = (arrow, "bold red on white")

    # Build Rich Text
    out = Text()
    for row in grid:
        for ch, style in row:
            out.append(ch, style=style)
        out.append("\n")

    # Footer with coordinates and legend
    axis_y_name = "Y" if plane == "xy" else "Z"
    footer = (
        f" [dim]X: [{min_x:+.1f}..{max_x:+.1f}]m  "
        f"{axis_y_name}: [{min_y:+.1f}..{max_y:+.1f}]m[/dim]\n"
        f" [cyan]·[/cyan] Ref Traj  [green]*[/green] NMPC Horizon (N={len(pred_pts)-1 if pred_pts is not None else 10})  [bold red]▲[/bold red] Drone"
    )
    out.append_text(Text.from_markup(footer))
    return out
