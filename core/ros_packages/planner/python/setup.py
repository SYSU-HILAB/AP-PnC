"""Setup script for the planning-core Python bindings.

Build with: uv run ap-pnc build planner-bindings
Compiles the pure planner_core sources directly (no colcon / no ROS needed).
Requires: pybind11, Eigen3 headers, yaml-cpp dev library.
"""

import os
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

from tooling.env import get_git_root

pkg = get_git_root() / "core" / "ros_packages" / "planner"
src = pkg / "src"


def _first_existing(paths: list[Path]) -> str | None:
    for p in paths:
        if p.exists():
            return str(p)
    return None


eigen_include = os.environ.get("EIGEN3_INCLUDE_DIR") or _first_existing(
    [Path("/usr/include/eigen3"), Path("/opt/homebrew/include/eigen3"),
     Path("/usr/local/include/eigen3")]
)

include_dirs = [
    str(src / "include"),
    str(src / "gcopter" / "include"),
    str(src / "basic_trajectories" / "include"),
    str(src / "tailsitter_df" / "include"),
]
if eigen_include:
    include_dirs.insert(0, eigen_include)

sources = [
    str(pkg / "python" / "planner_bindings.cpp"),
    str(src / "problem_config.cpp"),
    str(src / "planner.cpp"),
    str(src / "trajectory.cpp"),
    *[str(p) for p in sorted((src / "basic_trajectories" / "src").glob("*.cpp"))],
    *[str(p) for p in sorted((src / "tailsitter_df" / "src").glob("*.cpp"))],
]

ext_modules = [
    Pybind11Extension(
        "planner_bindings",
        sources=sources,
        include_dirs=include_dirs,
        libraries=["yaml-cpp"],
        extra_compile_args=["-O3"],
        cxx_std=17,
    ),
]

setup(
    name="planner_bindings",
    version="0.1.0",
    author="Erchao Rong, Zihao Liu, Junning Liang",
    author_email="rongerch@outlook.com",
    description="Python bindings for the AP-PnC pure planning core",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.8",
)
