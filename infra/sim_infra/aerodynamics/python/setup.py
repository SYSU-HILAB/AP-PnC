"""
Setup script for aerodynamics Python bindings.

Build with: uv run ap-pnc build bindings
(requires colcon-built aerodynamics model libs under .artifacts/colcon/build/)
"""

import os
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

# Build frontends pass absolute AP_PNC_DIR; fallback is anchored to this file, not cwd.
git_root = Path(os.environ.get("AP_PNC_DIR", Path(__file__).resolve().parents[4]))
if not git_root.is_absolute():
    raise ValueError("AP_PNC_DIR must be absolute")
git_root = git_root.resolve()
colcon_build = git_root / ".artifacts" / "colcon" / "build"
src_dir = git_root / "infra" / "sim_infra" / "aerodynamics"

ext_modules = [
    Pybind11Extension(
        "aerodynamics_bindings",
        sources=[str(src_dir / "src" / "pybind_aerodynamics.cpp")],
        include_dirs=[
            str(src_dir / "include"),
            str(src_dir / "src" / "MatlabGenerated" / "Zhang" / "include"),
            "/usr/include/eigen3",
        ],
        # deps/ carries the matching fmt/spdlog ABI from the ros2-base container,
        # listed first so the host's newer system fmt cannot shadow it.
        library_dirs=[
            str(git_root / ".artifacts" / "colcon" / "deps"),
            str(colcon_build / "aerodynamics"),
        ],
        extra_link_args=[f"-Wl,-rpath,{git_root / '.artifacts' / 'colcon' / 'deps'}"],
        libraries=[
            "aerodynamics_phi",
            "aerodynamics_lyu",
            "aerodynamics_non",
            "aerodynamics_advanced_lift_drag",
            "aerodynamics_bspline",
            "aerodynamics_aerodynamics_lib",
            "rt_nonfinite_matlab",
            "yaml-cpp",
            "fmt",
            "spdlog",
        ],
        extra_compile_args=["-O3"],
        cxx_std=17,
    ),
]

setup(
    name="aerodynamics_bindings",
    version="0.1.0",
    author="Erchao Rong, Zihao Liu, Junning Liang",
    author_email="rongerch@outlook.com",
    description="Python bindings for AP-PnC aerodynamics models",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.8",
)
