"""Local city staging and isolated Docker launch; no private data in fixtures."""

import json
import math
import shutil
from pathlib import Path

import pytest
import yaml

from tooling import city_demo
from tooling.env import get_git_root

ROOT = get_git_root()


@pytest.fixture
def project(tmp_path, monkeypatch):
    monkeypatch.setenv("AP_PNC_DIR", str(tmp_path))
    config = tmp_path / "core/bringup/config"
    config.mkdir(parents=True)
    for name in ("simple_sim.yaml", "planning.yaml", "nmpc.yaml"):
        shutil.copy2(ROOT / "core/bringup/config" / name, config / name)
    camera_dir = tmp_path / "infra/sim_infra/lidar_sim/web"
    camera_dir.mkdir(parents=True)
    shutil.copy2(
        ROOT / "infra/sim_infra/lidar_sim/web/camera_rig.json", camera_dir / "camera_rig.json"
    )
    profile = tmp_path / ".artifacts/vtol-matlab"
    profile.mkdir(parents=True)
    (profile / "profile.yaml").write_text(
        yaml.safe_dump(
            {
                "plant": {"mass": 2.12},
                "actuator": {
                    "body_frame": "FLU",
                    "enabled": True,
                    "test_fixture": True,
                    "kp": [0.01, 0.02, 0.03],
                    "kd": [0.001, 0.002, 0.003],
                },
            }
        )
    )
    (profile / "provenance.json").write_text("{}")
    for scene in ("city-loop", "city-ring"):
        assets = tmp_path / f".artifacts/dev-sim/{scene}/assets"
        assets.mkdir(parents=True)
        (assets / "city.ply").write_bytes(b"test fixture, not a Gaussian model")
        (assets / "occupancy.ply").write_bytes(b"test fixture, not a map")
        (assets / "scene.json").write_text("{}")
    executable = tmp_path / ".artifacts/dev-sim/city-loop/build/city_mission_node"
    executable.parent.mkdir(parents=True)
    executable.write_text("test executable fixture; never executed")
    return tmp_path


def test_frozen_closed_loop_and_profile(project):
    job = city_demo.prepare()
    mission = yaml.safe_load((job / "mission.yaml").read_text())
    assert mission["waypoints_m"][0] == mission["waypoints_m"][-1]
    assert len(mission["waypoints_m"]) == 16
    assert mission["scene_scale"] == 1 and mission["cruise_mps"] == 10
    assert mission["voxel_m"] * mission["dilation_cells"] == 2
    assert mission["accelerations_mps2"][0] == mission["accelerations_mps2"][-1] == [0, 0, 0]
    for p, v, a in zip(
        mission["waypoints_m"][1:-1],
        mission["velocities_mps"][1:-1],
        mission["accelerations_mps2"][1:-1],
        strict=True,
    ):
        assert math.hypot(p[0], p[1]) == pytest.approx(88.8)
        assert p[2] == 12
        assert p[0] * v[0] + p[1] * v[1] == pytest.approx(0, abs=1e-10)
        assert a == pytest.approx([-100 * p[0] / 88.8**2, -100 * p[1] / 88.8**2, 0])
    velocities = mission["velocities_mps"]
    assert velocities[0] == velocities[-1] == [0, 0, 0]
    assert all(sum(v * v for v in velocity) == pytest.approx(100) for velocity in velocities[1:-1])
    assert (
        project / ".artifacts/dev-sim/city-loop/assets/poses.json"
    ).resolve() == job / "stream/poses.json"
    assert (
        project / ".artifacts/dev-sim/city-loop/assets/verification.json"
    ).resolve() == job / "verification.json"
    view = json.loads((job / "view.json").read_text())
    assert view["scene_scale"] == 1 and view["synthetic_mirrored"]
    assert view["scene_asset"] == "/assets/ring-city.ply"
    assert (project / ".artifacts/dev-sim/city-loop/assets/ring-city.ply").resolve() == (
        project / ".artifacts/dev-sim/city-ring/assets/city.ply"
    )
    assert [camera["id"] for camera in view["camera_rig"]["cameras"]] == ["nose", "belly"]
    assert all(
        Path(mission[key]).is_absolute()
        for key in ("output", "planning", "simulation", "occupancy")
    )
    assert (job / "scene/occupancy.ply").read_bytes() == b"test fixture, not a map"
    sim = yaml.safe_load((job / "config/simple_sim.yaml").read_text())["simple_sim"]
    assert sim["plant"]["mass"] == 2.12 and sim["controller"] == "nmpc"
    assert sim["aero"]["model"] == "lyu"
    assert sim["actuator"]["body_frame"] == "FLU"
    assert sim["actuator"]["kp"] == pytest.approx([0.01, 0.02, 0.09])
    assert sim["actuator"]["kd"] == pytest.approx([0.001, 0.002, 0.009])
    assert yaml.safe_load((job / "profile.yaml").read_text())["actuator"]["kp"][0] == 0.01
    metadata = json.loads((job / "mission-metadata.json").read_text())
    assert metadata["schema"] == "ap-pnc/city-demo/v1"
    assert metadata["hardware_connected"] is False
    assert metadata["solver_variant"] == "baseline"
    assert metadata["inner_rate_tuning"] == "body-z-pd3"
    assert "awaits independent" in metadata["acceptance"]
    assert (job / "config/nmpc.yaml").read_text() == (
        project / "core/bringup/config/nmpc.yaml"
    ).read_text()
    assert len(metadata["asset_sha256"]["city.ply"]) == 64
    assert (
        project / ".artifacts/dev-sim/city-loop/assets/live.json"
    ).resolve() == job / "stream/live.json"


def test_original_scene_remains_explicitly_available(project):
    job = city_demo.prepare("original")
    mission = yaml.safe_load((job / "mission.yaml").read_text())
    assert len(mission["waypoints_m"]) == 10
    assert mission["scene_scale"] == 6
    assert "accelerations_mps2" not in mission


def test_baseline_rate_tuning_is_reproducible(project):
    job = city_demo.prepare(rate_tuning="baseline")
    sim = yaml.safe_load((job / "config/simple_sim.yaml").read_text())["simple_sim"]
    assert sim["actuator"]["kp"] == [0.01, 0.02, 0.03]
    assert sim["actuator"]["kd"] == [0.001, 0.002, 0.003]


def test_unknown_scene_rejected(project):
    with pytest.raises(ValueError, match="unknown city scene"):
        city_demo.prepare("unknown")


def test_missing_profile_stops_instead_of_substituting_ideal(project):
    (project / ".artifacts/vtol-matlab/profile.yaml").unlink()
    with pytest.raises(FileNotFoundError):
        city_demo.prepare()
    assert not (project / ".artifacts/dev-sim/city-loop/jobs").exists()


def test_launch_is_isolated_and_does_not_interpolate_paths_into_shell(project, monkeypatch):
    job = city_demo.prepare()
    calls = []
    monkeypatch.setattr(city_demo.subprocess, "run", lambda args, **kwargs: calls.append(args))
    city_demo.launch(job)
    args = calls[0]
    assert args[args.index("--network") + 1] == "none"
    assert "--ipc=host" in args
    assert "ROS_LOCALHOST_ONLY=1" in args and "ROS_DOMAIN_ID=213" in args
    assert str(job.name) not in args[args.index("-c") + 1]
    assert args[-1].endswith(f"/{job.name}/mission.yaml")
    assert any("recordings:/workspace/.artifacts/benchmark:rw" in arg for arg in args)
    assert (job / "executed-city_mission_node").is_file()
    receipt = json.loads((job / "execution.json").read_text())
    assert receipt["executable_sha256"] == city_demo.file_sha256(job / "executed-city_mission_node")
    assert receipt["frozen_files_sha256"]["mission.yaml"] == city_demo.file_sha256(
        job / "mission.yaml"
    )


@pytest.mark.parametrize("changed", ["mission.yaml", "config/nmpc.yaml"])
def test_relaunch_rejects_changed_inputs(project, monkeypatch, changed):
    job = city_demo.prepare()
    monkeypatch.setattr(city_demo.subprocess, "run", lambda *args, **kwargs: None)
    city_demo.launch(job)
    with (job / changed).open("a") as stream:
        stream.write("\n# changed after launch\n")
    with pytest.raises(ValueError, match="frozen execution inputs"):
        city_demo.launch(job)


def test_relaunch_rejects_changed_binary(project, monkeypatch):
    job = city_demo.prepare()
    monkeypatch.setattr(city_demo.subprocess, "run", lambda *args, **kwargs: None)
    city_demo.launch(job)
    (project / ".artifacts/dev-sim/city-loop/build/city_mission_node").write_text("another build")
    with pytest.raises(ValueError, match="another executable"):
        city_demo.launch(job)


def test_relative_launch_rejected_before_io(project):
    with pytest.raises(ValueError, match="absolute"):
        city_demo.launch(Path("relative-job"))
