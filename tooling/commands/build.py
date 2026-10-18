"""Build commands orchestrate Docker; compilers and Python bindings stay in Linux."""

import typer

from tooling import container_tools, simple_stack

app = typer.Typer()
ROOT = container_tools.CONTAINER_ROOT


def _colcon_build(
    packages: list[str],
    paths: list[str],
    *,
    clean: bool,
    arch: str,
    solver: bool = False,
) -> None:
    base = ROOT / ".artifacts/colcon"
    command = [
        "colcon", "--log-base", str(base / "log"), "build", "--merge-install",
        "--build-base", str(base / "build"), "--install-base", str(base / "install"),
        "--base-paths", *(str(ROOT / path) for path in paths),
    ]
    if packages:
        command.extend(["--packages-up-to", *packages])
    command.extend(["--cmake-args", "-DCMAKE_BUILD_TYPE=Release"])
    container_tools.run(command, arch=arch, clean=clean, solver=solver)


@app.command()
def aero(clean: bool = False, arch: str = "native") -> None:
    """Build aerodynamics in a Linux container."""
    _colcon_build(
        ["aerodynamics"], ["infra/sim_infra/aerodynamics"], clean=clean, arch=arch
    )


@app.command()
def planner(clean: bool = False, arch: str = "native") -> None:
    """Build the planner and its ROS dependencies in Docker."""
    _colcon_build(["planner"], ["core/ros_packages"], clean=clean, arch=arch)


@app.command(name="simple-sim")
def simple_sim() -> None:
    """Build the offline Docker pipeline: images, recorder and C++ simulator."""
    simple_stack.build()


@app.command()
def nmpc(clean: bool = False, arch: str = "native") -> None:
    """Build core NMPC in Docker (nav_infra is maintained separately)."""
    _colcon_build(["nmpc"], ["core/ros_packages"], clean=clean, arch=arch, solver=True)


@app.command(name="all")
def all_(clean: bool = False, arch: str = "native") -> None:
    """Build the core ROS workspace in Docker; excludes the separate nav_infra repo."""
    _colcon_build(
        [],
        ["core/ros_packages", "core/bringup", "infra/sim_infra/aerodynamics",
         "infra/sim_infra/simple_sim"],
        clean=clean, arch=arch, solver=True,
    )


def _bindings(script: str, arch: str) -> None:
    output = ROOT / ".artifacts/bindings"
    container_tools.run(
        ["python3", str(ROOT / script), "build_ext", "--build-lib", str(output),
         "--build-temp", str(output / "build")],
        arch=arch,
    )


@app.command()
def bindings(arch: str = "native") -> None:
    """Build Linux aerodynamics bindings under .artifacts/docker-tools/<arch>/."""
    aero(arch=arch)
    _bindings("infra/sim_infra/aerodynamics/python/setup.py", arch)


@app.command(name="planner-bindings")
def planner_bindings(arch: str = "native") -> None:
    """Build Linux planner bindings under .artifacts/docker-tools/<arch>/."""
    _bindings("core/ros_packages/planner/python/setup.py", arch)
