"""Offline-stack contracts; actual native Docker acceptance is separate."""

import importlib.util
import json
import os
import subprocess
from types import SimpleNamespace

import pytest
import tomllib

from tooling import simple_stack
from tooling.env import get_git_root
from tooling.paper.benchmark import stack_sweep
from tooling.simple_stack import JobRequest

ROOT = get_git_root()
STACK = ROOT / "infra/sim_infra/simple_sim/stack"


def test_offline_pipeline_runs_stages_in_order():
    source = (STACK / "src/main.rs").read_text()
    assert "let run = simulate(&path)?;" in source
    assert "let receipt = export(&run)?;" in source
    assert "verify(&receipt)?;" in source
    assert (
        source.index("let run = simulate(&path)?;")
        < source.index("let receipt = export(&run)?;")
        < source.index("verify(&receipt)?;")
    )
    assert not (STACK / "dataflow.yml").exists()


def test_versions_are_explicit_and_viewer_is_not_linked_into_sdk():
    manifest = tomllib.loads((STACK / "Cargo.toml").read_text())
    assert manifest["package"]["rust-version"] == "1.96"
    assert set(manifest["dependencies"]) == {
        "eyre",
        "serde",
        "serde_json",
        "serde_yaml",
        "csv",
        "sha2",
        "rerun",
    }
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
    requests = []

    def fake_run(request):
        requests.append(request)
        (tmp_path / "stack.log").write_text("complete")
        return tmp_path

    monkeypatch.setattr(
        module,
        "simple_stack",
        SimpleNamespace(
            JobRequest=JobRequest,
            run_job=fake_run,
            run_summary=lambda job: {"run_dir": job / "run"},
        ),
    )
    module.run(tmp_path, "practical", 0.0, vmax, 2.0, 2.0)
    index = json.loads((tmp_path / "index.json").read_text())
    assert index["v_max_setting_mps"] == vmax
    assert index["terminal_hold_s"] == 2.0
    assert index["mass_override_kg"] == 2.0
    assert len(requests) == 3
    assert {request.aero for request in requests} == {"none", "lyu", "phi"}
    assert all(
        (request.v_max, request.hold_s, request.mass_kg) == (vmax, 2.0, 2.0) for request in requests
    )


def test_rebuild_refreshes_simulator_and_model_parameter_image(monkeypatch):
    monkeypatch.setattr(simple_stack, "_image_exists", lambda name: True)
    commands = []
    simple_stack._ensure_stack_base_images(ROOT, "arm64", "linux/arm64", commands.append)
    assert len(commands) == 1
    assert commands[0][:6] == [
        "docker",
        "build",
        "--platform",
        "linux/arm64",
        "-t",
        "simple-sim:arm64",
    ]
    assert commands[0][-1] == str(ROOT)


def test_native_build_gives_ctest_an_isolated_writable_artifact_mount(tmp_path, monkeypatch):
    commands = []
    monkeypatch.setattr(simple_stack, "stack_root", lambda: tmp_path)
    monkeypatch.setattr(simple_stack, "setup_env", lambda: {})
    monkeypatch.setattr(simple_stack, "_ensure_stack_base_images", lambda *args: None)
    monkeypatch.setattr(
        simple_stack.subprocess, "run", lambda command, **kwargs: commands.append(command)
    )
    simple_stack.build()
    cpp = next(command for command in commands if command[-1] == simple_stack.CPP_STEPS)
    assert f"{tmp_path}/ctest-benchmark:/workspace/.artifacts/benchmark:rw" in cpp
    assert f"{ROOT}:/workspace:ro" in cpp


def test_runtime_does_not_mount_over_image_install():
    driver = (STACK / "stack.sh").read_text()
    assert '"$job/benchmark:/workspace/.artifacts/benchmark:rw"' in driver
    assert "--ipc=host" in driver
    assert '--platform "linux/$arch"' in driver
    assert '"$AP_PNC_DIR/.artifacts:/workspace/.artifacts' not in driver
    assert '"verified.json"' in (STACK / "src/main.rs").read_text()


def test_same_second_jobs_never_overwrite_scientific_data(tmp_path, monkeypatch):
    monkeypatch.setattr(simple_stack, "stack_root", lambda: tmp_path)
    monkeypatch.setattr(simple_stack.time, "strftime", lambda *args: "same-second")
    monkeypatch.setattr(simple_stack, "_write_metadata", lambda job: None)

    def fake_container(job, *args):
        folder = job / "benchmark/simple_sim_1_0"
        folder.mkdir()
        (folder / "verified.json").write_text("{}")
        return SimpleNamespace(returncode=0, stdout="verified", stderr="")

    monkeypatch.setattr(simple_stack, "_container", fake_container)
    first, second = simple_stack.run_job(), simple_stack.run_job()
    assert first != second
    assert (first / "benchmark/simple_sim_1_0/verified.json").is_file()
    with pytest.raises(FileExistsError):
        simple_stack.run_job(job=first)


def test_failed_sweep_produces_error_row_instead_of_crashing(tmp_path, monkeypatch):
    def reject(**kwargs):
        raise RuntimeError("test fixture failure")

    monkeypatch.setattr(stack_sweep, "_run_once", reject)
    rows = stack_sweep.run_matrix(
        aeros=("lyu",),
        v_maxs=(4.0,),
        variants=("nmpc",),
        profiles=("practical",),
        output_dir=tmp_path,
    )
    assert len(rows) == 1 and rows[0].status == "error"
    assert (tmp_path / "sweep.json").is_file()
