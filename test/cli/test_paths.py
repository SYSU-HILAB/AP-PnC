"""Project paths must not depend on the shell's working directory."""

import subprocess
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tooling import docker_runtime, docker_runtime as docker, env
from tooling.commands import lint, test as test_command
from tooling.paper.benchmark import data
from tooling.paper.figure import data_utils, specify_figure_dirs


@pytest.fixture
def roots(tmp_path, monkeypatch):
    project = tmp_path / "project"
    foreign = tmp_path / "elsewhere"
    project.mkdir()
    foreign.mkdir()
    monkeypatch.setenv("AP_PNC_DIR", str(project))
    monkeypatch.chdir(foreign)
    return project, foreign


def test_discovery_ignores_foreign_repository(tmp_path, monkeypatch):
    expected = Path(env.__file__).resolve().parents[1]
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    subprocess.run(["git", "-C", str(foreign), "init", "-q"], check=True)
    monkeypatch.chdir(foreign)
    monkeypatch.delenv("AP_PNC_DIR", raising=False)
    assert env.get_git_root() == expected


@pytest.mark.parametrize("value", ["", "relative", "../project", "/nonexistent-ap-pnc-test-root"])
def test_invalid_root_is_not_resolved_against_cwd(roots, monkeypatch, value):
    monkeypatch.setenv("AP_PNC_DIR", value)
    with pytest.raises(ValueError, match="absolute project root"):
        env.get_git_root()


def test_missing_checkout_requires_explicit_root(roots, monkeypatch):
    monkeypatch.delenv("AP_PNC_DIR")

    def failed_git(args, **kwargs):
        assert args[:2] == ["git", "-C"]
        assert Path(args[2]).is_absolute()
        raise subprocess.CalledProcessError(128, args)

    monkeypatch.setattr(env.subprocess, "run", failed_git)
    with pytest.raises(RuntimeError, match="set absolute AP_PNC_DIR"):
        env.get_git_root()


def test_worktree_discovery_is_module_anchored(roots, monkeypatch):
    project, foreign = roots
    git_dir = project / ".git" / "worktrees" / "test"
    git_dir.mkdir(parents=True)
    (git_dir / "commondir").write_text("../..\n")
    monkeypatch.delenv("AP_PNC_DIR")
    monkeypatch.setattr(env, "__file__", str(foreign / "tooling" / "env.py"))

    def git(args, **kwargs):
        assert args[:3] == ["git", "-C", str(foreign)]
        return SimpleNamespace(stdout=str(git_dir))

    monkeypatch.setattr(env.subprocess, "run", git)
    assert env.get_git_root() == project


def test_inputs_are_root_anchored_and_outputs_are_confined(roots):
    project, foreign = roots
    assert env.project_path("core/bringup/config/planning.yaml") == project / "core/bringup/config/planning.yaml"
    assert env.project_path(foreign / "input.csv") == foreign / "input.csv"
    assert env.artifact_path(".artifacts/results/rows.npz") == project / ".artifacts/results/rows.npz"
    for value in ("../escape", "core/results.json", foreign / "result.json"):
        with pytest.raises(ValueError):
            env.artifact_path(value)


def test_output_symlink_cannot_escape_artifacts(roots):
    project, foreign = roots
    artifacts = project / ".artifacts"
    artifacts.mkdir()
    (artifacts / "escape").symlink_to(foreign, target_is_directory=True)
    with pytest.raises(ValueError, match="Generated output"):
        env.artifact_path(".artifacts/escape/result.json")


def test_pytest_operands_and_caches_are_absolute(roots, monkeypatch):
    project, _ = roots
    calls = []
    monkeypatch.setattr(test_command, "setup_env", lambda: None)
    monkeypatch.setattr(test_command.subprocess, "run", lambda args, **kw: calls.append((args, kw)))
    test_command._pytest("test/cli")
    args, options = calls[0]
    assert args[3] == str(project / "test/cli")
    assert args[args.index("--basetemp") + 1] == str(project / ".artifacts/tests/pytest_tmp")
    assert f"cache_dir={project / '.artifacts/tests/pytest_cache'}" in args
    assert options["cwd"] == project
    assert options["env"]["PYTHONDONTWRITEBYTECODE"] == "1"


@pytest.mark.parametrize("command, operation", [(lint.lint, "check"), (lint.fmt, "format")])
def test_ruff_operands_and_cache_are_absolute(roots, monkeypatch, command, operation):
    project, _ = roots
    calls = []
    monkeypatch.setattr(lint, "setup_env", lambda: None)
    monkeypatch.setattr(lint.subprocess, "run", lambda args, **kw: calls.append((args, kw)))
    command()
    args, options = calls[0]
    assert args[3:5] == [operation, str(project / "test")]
    assert args[args.index("--cache-dir") + 1] == str(project / ".artifacts/ruff_cache")
    assert options["cwd"] == project


def test_compose_has_explicit_project_environment(roots, monkeypatch):
    project, _ = roots
    compose_file = project / "infra/sim_infra/gazebo/docker/docker-compose.yml"
    compose_file.parent.mkdir(parents=True)
    compose_file.write_text("services:\n  ap-pnc:\n    image: ap-pnc:arm64\n")
    calls = []
    monkeypatch.setattr(docker_runtime.subprocess, "run", lambda args, **kw: calls.append((args, kw)))
    docker._compose("config")
    args, options = calls[0]
    assert Path(args[3]).is_absolute()
    assert options["env"]["AP_PNC_DIR"] == str(project)


def test_benchmark_io_and_figure_cache_do_not_write_in_cwd(roots):
    project, foreign = roots
    rows = {"t": np.array([0.0, 1.0]), "p": np.zeros((2, 3))}
    npz = data.save_npz(".artifacts/benchmark/path-test/rows.npz", rows, {"case": "paths"})
    data.save_json(".artifacts/benchmark/path-test/metrics.json", {"ok": True})
    data.to_csv(".artifacts/benchmark/path-test/rows.csv", rows)
    loaded, meta = data.load_npz(".artifacts/benchmark/path-test/rows.npz")
    assert npz.is_relative_to(project / ".artifacts")
    np.testing.assert_array_equal(loaded["p"], rows["p"])
    assert meta == {"case": "paths"}
    cache = Path(data_utils.save_aero_data(rows, "test.npy"))
    assert cache == project / ".artifacts/paper/cache/test.npy"
    assert Path(specify_figure_dirs.tracking_plots_dir()) == project / ".artifacts/paper/figures/tracking"
    with pytest.raises(ValueError):
        data.save_json("core/metrics.json", {})
    with pytest.raises(ValueError):
        data_utils.get_data_path("../../../../core/escape.npy")
    assert not list(foreign.iterdir())
    assert not (project / "core").exists()
