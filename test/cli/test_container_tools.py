"""Every build/test entrypoint dispatches Linux tools through Docker on any host."""

from pathlib import Path
from unittest.mock import Mock

import pytest

from tooling import container_tools, env
from tooling.commands import build, test as test_commands


@pytest.mark.parametrize("arch", ["arm64", "amd64"])
def test_tools_only_run_docker_with_target_isolation(tmp_path, monkeypatch, arch):
    run, bundle = Mock(), Mock()
    monkeypatch.setattr(container_tools, "get_git_root", lambda: tmp_path)
    monkeypatch.setattr(container_tools.subprocess, "run", run)
    monkeypatch.setattr(container_tools, "ensure_bundle", bundle)
    container_tools.run(["colcon", "build"], arch=arch, solver=True)
    bundle.assert_called_once_with(tmp_path, arch)
    commands = [call.args[0] for call in run.call_args_list]
    assert [command[:2] for command in commands] == [["docker", "build"], ["docker", "run"]]
    assert all(command[command.index("--platform") + 1] == f"linux/{arch}" for command in commands)
    command = commands[-1]
    assert "--ipc=host" in command
    assert f"{tmp_path}:/workspace:ro" in command
    assert f"{tmp_path}/.artifacts/docker-tools/{arch}:/workspace/.artifacts:rw" in command
    assert command[-2:] == ["colcon", "build"]
    assert "PYTHONDONTWRITEBYTECODE=1" in command


@pytest.mark.parametrize("name", ["aero", "planner", "nmpc", "all_", "bindings", "planner_bindings"])
def test_build_commands_never_invoke_host_compilers(monkeypatch, name):
    run = Mock()
    monkeypatch.setattr(container_tools, "run", run)
    getattr(build, name)(arch="arm64")
    assert run.called
    for call in run.call_args_list:
        args = call.args[0]
        assert args[0] in ("python3", "colcon")
        assert call.kwargs["arch"] == "arm64"
        for arg in args:
            if arg.startswith("/"):
                assert Path(arg).is_relative_to("/workspace")
    if name in ("nmpc", "all_"):
        assert run.call_args.kwargs["solver"]


@pytest.mark.parametrize(
    "name", ["all_", "vmap", "jax_trim", "casadi", "frame_conversion", "colcon", "colcon_result"]
)
def test_numerical_test_commands_only_dispatch_containers(monkeypatch, name):
    run = Mock()
    monkeypatch.setattr(container_tools, "run", run)
    getattr(test_commands, name)(arch="amd64")
    run.assert_called_once()
    assert run.call_args.kwargs["arch"] == "amd64"


def test_environment_setup_never_downloads_or_loads_host_toolchains(tmp_path, monkeypatch):
    monkeypatch.setenv("AP_PNC_DIR", str(tmp_path))
    monkeypatch.delenv("ACADOS_SOURCE_DIR", raising=False)
    monkeypatch.delenv("ACADOS_PYTHON_INTERFACE_PATH", raising=False)
    monkeypatch.delenv("LD_LIBRARY_PATH", raising=False)
    run = Mock(side_effect=AssertionError("environment setup must not run tools"))
    monkeypatch.setattr(env.subprocess, "run", run)
    env.setup_env()
    run.assert_not_called()
    assert "ACADOS_SOURCE_DIR" not in env.os.environ
    assert "ACADOS_PYTHON_INTERFACE_PATH" not in env.os.environ
    assert "LD_LIBRARY_PATH" not in env.os.environ
