"""Offline-stack contracts; actual native Docker acceptance is separate."""

import importlib.util
import json
import os
import subprocess
from types import SimpleNamespace

import pytest
import tomllib
import yaml

from tooling.env import get_git_root

ROOT = get_git_root()
STACK = ROOT / "infra/sim_infra/simple_sim/stack"


def test_graph_only_transports_finished_run_references():
    graph = yaml.safe_load((STACK / "dataflow.yml").read_text())
    nodes = {n["id"]: n for n in graph["nodes"]}
    assert set(nodes) == {"simulate", "record", "verify"}
    assert nodes["record"]["inputs"] == {"run": "simulate/run"}
    assert nodes["verify"]["inputs"] == {"recording": "record/recording"}
    for node in nodes.values():
        assert node["path"].startswith("/workspace/.artifacts/")
        assert "timer" not in str(node)
        assert "codegen" not in str(node)


def test_versions_are_explicit_and_viewer_is_not_linked_into_sdk():
    manifest = tomllib.loads((STACK / "Cargo.toml").read_text())
    assert manifest["package"]["rust-version"] == "1.96"
    assert manifest["dependencies"]["dora-node-api"]["version"] == "=1.0.1"
    sdk = manifest["dependencies"]["rerun"]
    assert sdk == {"version": "=0.38.1", "default-features": False, "features": ["sdk"]}
    docker = (STACK / "Dockerfile").read_text()
    assert "rust:1.96.0-slim-bookworm@sha256:" in docker
    assert "sha256sum -c -" in docker
    assert (STACK / "Cargo.lock").is_file()


def test_driver_rejects_relative_root_without_docker(tmp_path):
    env = dict(os.environ, AP_PNC_DIR="relative/root")
    result = subprocess.run(
        ["bash", str(STACK / "stack.sh"), "run"],
        env=env,
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "invalid AP_PNC_DIR" in result.stderr


def test_driver_rejects_foreign_job_without_docker(tmp_path):
    env = dict(os.environ, AP_PNC_DIR=str(ROOT))
    result = subprocess.run(
        ["bash", str(STACK / "stack.sh"), "export", str(tmp_path), "simple_sim_1_0"],
        env=env,
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "invalid job directory" in result.stderr


@pytest.mark.parametrize("vmax", [8.0, 10.0, 12.0])
def test_sweep_passes_actual_speed_and_terminal_hold(vmax, tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("tracking_evaluate", STACK / "evaluate.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    commands = []

    def fake_run(argv, **kwargs):
        commands.append(argv)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(
        module, "subprocess", SimpleNamespace(run=fake_run, STDOUT=subprocess.STDOUT)
    )
    module.run(tmp_path, "practical", 0.0, vmax, 2.0, 2.0)
    index = json.loads((tmp_path / "index.json").read_text())
    assert index["v_max_setting_mps"] == vmax
    assert index["terminal_hold_s"] == 2.0
    assert index["mass_override_kg"] == 2.0
    assert len(commands) == 3
    assert {command[5] for command in commands} == {"none", "lyu", "phi"}
    assert all(cmd[-3:] == [str(vmax), "2.0", "2.0"] for cmd in commands)


def test_runtime_does_not_mount_over_image_install():
    driver = (STACK / "stack.sh").read_text()
    assert '"$job/benchmark:/workspace/.artifacts/benchmark:rw"' in driver
    assert "--ipc=host" in driver
    assert '--platform "linux/$arch"' in driver
    assert '"$AP_PNC_DIR/.artifacts:/workspace/.artifacts' not in driver
    assert '"verified.json"' in (STACK / "src/main.rs").read_text()
