"""
Setup script for aerodynamics Python bindings.

Build with: ./pyp -m pip install -e --no-build-isolation src/aerodynamics/python/
"""

from pathlib import Path

# Find directories
git_root = Path(__file__).parent.parent.parent.parent
build_dir = git_root / "build" / "aerodynamics"
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
            str(git_root / "src" / "utils" / "external_libraries" / "BSplinebasis" / "include"),
        ],
        library_dirs=[
            str(build_dir),
            str(git_root / "build" / "utils"),
        ],
        libraries=[
            "aerodynamics_phi",
            "aerodynamics_lyu",
            "aerodynamics_non",
            "aerodynamics_advanced_lift_drag",
            "aerodynamics_bspline",
            "aerodynamics_aerodynamics_lib",
            "utils_config_reader",
            "utils_config_reader_phi",
            "utils_config_reader_advanced_lift_drag",
            "utils_config_reader_bspline",
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
