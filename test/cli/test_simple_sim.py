from pathlib import Path
from unittest.mock import Mock

import pytest
import typer

from tooling.commands import build, sim


def test_build_selects_sim_and_bringup_once(tmp_path, monkeypatch):
    run = Mock()
    monkeypatch.setattr(build, "get_git_root", lambda: tmp_path)
    monkeypatch.setattr(build, "_colcon_env", lambda: {})
    monkeypatch.setattr(build.subprocess, "run", run)
    build.simple_sim(clean=False)
    args = run.call_args.args[0]
    selected = args.index("--packages-select")
    assert args[selected : selected + 3] == ["--packages-select", "simple_sim", "bringup"]
    assert args.count("--packages-select") == 1
    paths = args.index("--base-paths")
    assert args[paths + 1 : paths + 3] == [
        str(tmp_path / "infra/sim_infra/simple_sim"),
        str(tmp_path / "core/bringup"),
    ]
    assert "-DSIMPLE_SIM_WITH_ROS=ON" in args
    assert "-DSIMPLE_SIM_WITH_NMPC=ON" in args
    assert run.call_args.kwargs["env"]["COLCON_LOG_PATH"] == str(tmp_path / ".artifacts/colcon/log")


def installed_sim(root: Path) -> Path:
    executable = root / ".artifacts/colcon/install/lib/simple_sim/simple_sim_run"
    executable.parent.mkdir(parents=True)
    executable.write_text("test executable placeholder")
    return executable


def test_track_invokes_absolute_installed_executable(tmp_path, monkeypatch):
    executable = installed_sim(tmp_path)
    run = Mock()
    monkeypatch.setattr(sim, "get_git_root", lambda: tmp_path)
    monkeypatch.setattr(sim.subprocess, "run", run)
    config = tmp_path / "config.yaml"
    sim.track(config=config, controller="se3")
    assert run.call_args.args[0] == [
        str(executable),
        "--config",
        str(config),
        "--controller",
        "se3",
    ]
    assert run.call_args.kwargs["env"]["AP_PNC_DIR"] == str(tmp_path)
    assert run.call_args.kwargs["check"] is True


def test_track_rejects_relative_config_and_invalid_controller(tmp_path, monkeypatch):
    installed_sim(tmp_path)
    monkeypatch.setattr(sim, "get_git_root", lambda: tmp_path)
    with pytest.raises(typer.BadParameter, match="absolute"):
        sim.track(config=Path("config.yaml"))
    with pytest.raises(typer.BadParameter, match="nmpc or se3"):
        sim.track(controller="invalid")


def test_track_requires_build(tmp_path, monkeypatch):
    monkeypatch.setattr(sim, "get_git_root", lambda: tmp_path)
    with pytest.raises(typer.BadParameter, match="not installed"):
        sim.track()
