# BSD 3-Clause License
#
# Copyright (c) 2025 - <YEARS>, Sun Yat-sen University.
# All rights reserved.
#
# Authors:
# Erchao Rong: rongerch@outlook.com
# Zihao Liu: liuzh297@gmail.com
# Junning Liang: gordonliang27@foxmail.com
#
# Paper:
# Aerodynamic Prior-free Trajectory Generation and Tracking Control for a Tail-sitter UAV.

from pathlib import Path

from tooling.env import artifact_path, get_git_root


def figure_path(filename):
    """Resolve a figure filename inside the artifact-owned figure directory."""
    path = artifact_path(Path(".artifacts/paper/figures") / filename)
    path.parent.mkdir(parents=True, exist_ok=True)
    return str(path)


def root_dir():
    """Return the absolute project root, never an optional cwd fallback."""
    return str(get_git_root())


def tracking_plots_dir():
    """Specify the directories for the tracking data."""

    tracking_dir = artifact_path(".artifacts/paper/figures/tracking")
    tracking_dir.mkdir(parents=True, exist_ok=True)
    return str(tracking_dir)
