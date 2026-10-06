"""Planner source: run the real C++ planning core through pybind11.

This replaces the removed CSV/TrajectoryReference interface: the benchmark
calls the pure planning core in-process and gets a typed Reference back.
"""

from __future__ import annotations

import sys
from pathlib import Path

from tooling.env import get_git_root, project_path

from ..reference import Reference


def _ensure_bindings_on_path() -> None:
    """Add the pybind11 build output dir to sys.path."""
    bindings_dir = get_git_root() / ".artifacts" / "bindings"
    if bindings_dir.is_dir() and str(bindings_dir) not in sys.path:
        sys.path.insert(0, str(bindings_dir))


def plan(yaml_path: str | Path) -> Reference:
    """Plan a reference trajectory from a planning yaml via planner_bindings.

    Raises:
        RuntimeError: if the planner bindings have not been built.
    """
    _ensure_bindings_on_path()
    try:
        import planner_bindings  # type: ignore
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError(
            "planner_bindings is not available. Build it with "
            "`uv run ap-pnc build planner-bindings`."
        ) from exc

    data = planner_bindings.plan(str(project_path(yaml_path)))
    return Reference(
        t=data["t"],
        p=data["p"],
        v=data["v"],
        a=data["a"],
        yb=data["yb"],
        omega=data["omega"],
        thrust=data["thrust"],
    )
