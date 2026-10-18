"""Planner and NMPC share the src/node/optional-python package contract."""

import runpy
from pathlib import Path

import pytest
import setuptools

from tooling.commands import gen_nmpc_lib
from tooling.env import get_git_root


@pytest.mark.parametrize("package", ["planner", "nmpc"])
def test_cpp_and_ros_layers_share_the_package_layout(package):
    root = get_git_root() / "core" / "ros_packages" / package
    assert (root / "src" / "CMakeLists.txt").is_file()
    assert list((root / "src").glob("*.cpp"))
    assert (root / "node" / "src" / f"{package}_node.cpp").is_file()
    for old_directory in ("core", "solver", "controller"):
        assert not (root / old_directory).exists()
    for source in (root / "src").rglob("*.cpp"):
        assert "#include <rclcpp" not in source.read_text()
        assert '#include "rclcpp' not in source.read_text()


@pytest.mark.parametrize(
    "message, fields",
    [
        ("State", ["uint64 timestamp", "float64[13] state"]),
        (
            "Control",
            [
                "uint64 timestamp",
                "float64[3] rates_sp",
                "float64 specific_force_sp",
                "float64 sol_time",
            ],
        ),
        ("ReferencePoint", ["float64[3] pos", "float64[3] vel", "float64[3] yb"]),
        (
            "TrackingInfo",
            [
                "uint64 timestamp",
                "float32[3] ref_position",
                "float32[3] ref_velocity",
                "float32[3] actual_position",
                "float32[3] actual_velocity",
                "float32 ref_vxy",
                "float32 actual_vxy",
                "float32[3] ref_yb",
                "float32[3] actual_yb",
                "float32 alpha",
                "float32 beta",
                "float32 cx",
            ],
        ),
    ],
)
def test_quadrotor_message_schema(message, fields):
    path = get_git_root() / "core/ros_packages/interface/msg" / f"{message}.msg"
    actual = []
    for line in path.read_text().splitlines():
        declaration = " ".join(line.split("#", 1)[0].split())
        if declaration and "=" not in declaration:
            actual.append(declaration)
    assert actual == fields


def test_planner_bindings_compile_sources_from_src(monkeypatch, tmp_path):
    root = get_git_root() / "core" / "ros_packages" / "planner"
    captured = {}
    monkeypatch.setattr(setuptools, "setup", lambda **kwargs: captured.update(kwargs))
    monkeypatch.chdir(tmp_path)
    runpy.run_path(str(root / "python" / "setup.py"))
    extension = captured["ext_modules"][0]
    assert str(root / "src" / "planner.cpp") in extension.sources
    assert str(root / "src" / "trajectory.cpp") in extension.sources
    for source in (root / "src").glob("*.cpp"):
        assert str(source) in extension.sources
    assert str(root / "src" / "include") in extension.include_dirs
    for source in extension.sources:
        assert Path(source).is_absolute()
        assert Path(source).is_file()


def test_bundle_builder_configures_the_src_layer(monkeypatch, tmp_path):
    def configure(args, **_kwargs):
        assert args[:3] == [
            "cmake",
            "-S",
            str(tmp_path / "core" / "ros_packages" / "nmpc" / "src"),
        ]
        raise RuntimeError("configuration inspected")

    monkeypatch.setattr(gen_nmpc_lib.subprocess, "run", configure)
    with pytest.raises(RuntimeError, match="configuration inspected"):
        gen_nmpc_lib._build_wrapper_and_bundle(
            tmp_path,
            arch="aarch64",
            staging_root=Path("/opt/acados"),
            static_dir=Path("/opt/acados_static"),
        )
