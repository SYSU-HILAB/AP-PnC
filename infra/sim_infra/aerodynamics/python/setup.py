"""
Setup script for aerodynamics Python bindings.

Build with: uv run ap-pnc build bindings
(requires colcon-built aerodynamics/utils libs under .artifacts/colcon/build/)
"""

from pathlib import Path

# Find directories
git_root = Path(__file__).parent.parent.parent.parent
colcon_build = git_root / ".artifacts" / "colcon" / "build"
src_dir = Path(__file__).parent.parent

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

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
            "sqlite3",
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
    author="Hanamy",
    description="Python bindings for AP-PnC aerodynamics models",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.8",
)
