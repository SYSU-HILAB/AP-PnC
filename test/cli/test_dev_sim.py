"""Staging contracts: practical+lyu, isolated ROS domain and no benchmark runs."""

import json
import shutil
from pathlib import Path

import pytest
import yaml

from tooling import dev_sim
from tooling.env import get_git_root

ROOT = get_git_root()


@pytest.fixture
def project(tmp_path, monkeypatch):
    monkeypatch.setenv("AP_PNC_DIR", str(tmp_path))
    config = tmp_path / "core/bringup/config"
    config.mkdir(parents=True)
    for name in ("simple_sim.yaml", "planning.yaml", "nmpc.yaml", "lidar_sim.yaml", "lidar_sim_3dgs_garden.yaml"):
        shutil.copy2(ROOT / "core/bringup/config" / name, config / name)
    profiles = tmp_path / ".artifacts/vtol-matlab"
    profiles.mkdir(parents=True)
    (profiles / "profile.yaml").write_text(yaml.safe_dump({
        "plant": {"mass": 2.12, "inertia": [0.01, 0.02, 0.03]},
        "actuator": {"enabled": True, "test_fixture": True},
    }))
    (profiles / "provenance.json").write_text("{}")
    monkeypatch.setattr(dev_sim.subprocess, "check_output", lambda *args, **kwargs: "fixture-revision")
    return tmp_path


def test_prepared_configuration_is_practical_lyu_and_frozen(project):
    job = dev_sim.prepare(output=project / ".artifacts/dev-test")
    sim = yaml.safe_load((job / "config/simple_sim.yaml").read_text())["simple_sim"]
    plan = yaml.safe_load((job / "config/planning.yaml").read_text())["problem_formulation"]
    assert sim["aero"]["model"] == "lyu" and sim["actuator"]["enabled"]
    assert sim["controller"] == "nmpc" and sim["plant"]["mass"] == 2.12
    assert plan["flatness"]["mass"] == 2.12 and plan["basic_traj"]["height"] == 3
    assert plan["basic_traj"]["radius"] == 8 and plan["cost"]["v_max"] == 4
    assert (job / "recordings").is_dir() and not (job / "benchmark").exists()
    args = dev_sim.command(job)
    assert args[args.index("--network") + 1] == "none"
    assert "ROS_DOMAIN_ID=213" in args and "ROS_LOCALHOST_ONLY=1" in args
    assert "lidar_config:=/workspace/core/bringup/config/lidar_sim.yaml" in args
    assert json.loads((job / "dev-sim.json").read_text())["hardware_connected"] is False
    with pytest.raises(FileExistsError):
        dev_sim.prepare(output=job)


@pytest.mark.parametrize("speed", [float("inf"), float("nan"), -1, 0])
def test_invalid_speed_has_no_partial_job(project, speed):
    output = project / ".artifacts/invalid"
    with pytest.raises(ValueError):
        dev_sim.prepare(v_max=speed, output=output)
    assert not output.exists()


def test_missing_private_profile_never_falls_back_to_ideal(project):
    (project / ".artifacts/vtol-matlab/profile.yaml").unlink()
    with pytest.raises(FileNotFoundError, match="practical profile unavailable"):
        dev_sim.prepare()


def test_generated_outputs_cannot_escape_artifacts(project):
    with pytest.raises(ValueError, match=".artifacts"):
        dev_sim.prepare(output=project / "generated")


def test_relative_job_is_rejected_before_io(project):
    # Command inputs must be absolute, even when cwd points at this checkout.
    with pytest.raises(ValueError, match="absolute"):
        dev_sim.command(Path(".artifacts/job"))


def test_unknown_scene_is_rejected(project):
    with pytest.raises(ValueError, match="scene"):
        dev_sim.prepare(scene="bogus", output=project / ".artifacts/bogus")


def test_garden_3dgs_scene_requires_converted_points_first(project):
    with pytest.raises(FileNotFoundError, match="garden-3dgs"):
        dev_sim.prepare(scene="garden-3dgs", output=project / ".artifacts/garden")


def test_garden_3dgs_scene_mounts_points_and_freezes_point_cloud_config(project):
    scene_dir = project / ".artifacts/dev-sim/scenes"
    scene_dir.mkdir(parents=True)
    points = scene_dir / "garden_points.ply"
    points.write_bytes(b"ply\nformat binary_little_endian 1.0\nelement vertex 0\n"
                       b"property float x\nproperty float y\nproperty float z\nend_header\n")
    job = dev_sim.prepare(scene="garden-3dgs", output=project / ".artifacts/dev-garden", v_max=10.0)
    lidar = yaml.safe_load((job / "config/lidar_sim.yaml").read_text())["lidar_sim"]
    assert lidar["scene"]["type"] == "point_cloud"
    assert (job / "scene/garden_points.ply").is_file()
    args = dev_sim.command(job)
    assert f"{job}/scene:/workspace/scene:ro" in args
    assert json.loads((job / "dev-sim.json").read_text())["scene"] == "garden-3dgs"
