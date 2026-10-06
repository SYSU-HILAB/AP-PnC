"""Multiarch targeting must not depend on cwd, host aliases or other builds' caches."""

import json
import shutil
import subprocess
from pathlib import Path
from unittest.mock import Mock

import pytest
import typer
import yaml

from tooling import docker_runtime as runtime
from tooling.commands import docker, gen_nmpc_lib, sim, stack
from tooling.env import get_git_root

ROOT = get_git_root()


@pytest.mark.parametrize(
    ("value", "expected"),
    [("arm64", "arm64"), ("aarch64", "arm64"), ("amd64", "amd64"), ("x86_64", "amd64")],
)
def test_arch_aliases(value, expected):
    assert runtime.target_arch(value) == expected


@pytest.mark.parametrize("machine", ["arm64", "aarch64", "x86_64", "AMD64"])
def test_native_is_host_cpu_not_an_amd64_fallback(monkeypatch, machine):
    monkeypatch.setattr(runtime.platform, "machine", lambda: machine)
    assert runtime.target_arch() == runtime.ARCH_ALIASES[machine.lower()]


def test_bad_target_is_rejected():
    with pytest.raises(typer.BadParameter):
        runtime.target_arch("riscv64")
    with pytest.raises(typer.BadParameter):
        runtime.compose_env(ROOT, "arm64", "typo")
    with pytest.raises(ValueError, match="absolute"):
        runtime.compose_env(Path("relative"))


def test_compose_native_environment_and_profiles(tmp_path, monkeypatch):
    run = Mock()
    monkeypatch.setattr(runtime.subprocess, "run", run)
    monkeypatch.setattr(runtime.platform, "machine", lambda: "arm64")
    monkeypatch.chdir(tmp_path)
    file = ROOT / "infra/sim_infra/gazebo/docker/docker-compose.yml"
    runtime.compose(ROOT, file, ["build", "ap-pnc"], gui=True)
    args = run.call_args.args[0]
    assert args[:4] == ["docker", "compose", "-f", str(file)]
    assert args[4:] == ["--profile", "build", "--profile", "gui", "build", "ap-pnc"]
    options = run.call_args.kwargs
    assert options["env"]["AP_PNC_DIR"] == str(ROOT)
    assert options["env"]["AP_PNC_ARCH"] == "arm64"
    assert options["cwd"] == ROOT
    assert options["check"]


def test_down_includes_gui_but_not_build_profile(monkeypatch):
    run = Mock()
    monkeypatch.setattr(runtime.subprocess, "run", run)
    docker.down(arch="amd64")
    args = run.call_args.args[0]
    assert args[4:] == ["--profile", "gui", "down"]
    assert run.call_args.kwargs["env"]["AP_PNC_ARCH"] == "amd64"


def test_build_validates_before_generating_solver(monkeypatch):
    bundle = Mock()
    monkeypatch.setattr(docker, "ensure_bundle", bundle)
    with pytest.raises(typer.BadParameter):
        docker.build(arch="arm64", nav_target="typo")
    with pytest.raises(typer.BadParameter):
        docker.build(services=["typo"])
    bundle.assert_not_called()


def test_gui_only_build_does_not_require_solver(monkeypatch):
    bundle, compose = Mock(), Mock()
    monkeypatch.setattr(docker, "ensure_bundle", bundle)
    monkeypatch.setattr(docker, "_compose", compose)
    docker.build(services=["desktop"], arch="aarch64")
    bundle.assert_not_called()
    assert compose.call_args.kwargs["gui"] is True


def test_bundle_never_accepts_unsuffixed_alias(tmp_path, monkeypatch):
    solver = tmp_path / ".artifacts/nmpc_solver"
    solver.mkdir(parents=True)
    (solver / "libnmpc_bundle.a").write_bytes(b"wrong target")
    calls = []

    def generate(*, arch):
        calls.append(arch)
        (solver / f"libnmpc_bundle_{arch}.a").write_bytes(b"selected target")

    monkeypatch.setattr(gen_nmpc_lib, "cocp", generate)
    runtime.ensure_bundle(tmp_path, "arm64")
    runtime.ensure_bundle(tmp_path, "arm64")
    assert calls == ["aarch64"]


def test_cross_builder_caches_are_isolated_and_host_alias_preserved(tmp_path, monkeypatch):
    solver = tmp_path / ".artifacts/nmpc_solver"
    solver.mkdir(parents=True)
    alias = solver / "libnmpc_bundle.a"
    alias.write_bytes(b"host x86")
    calls = []

    def run(args, **kwargs):
        calls.append(args)
        if args[:2] == ["docker", "run"]:
            build_solver = tmp_path / ".artifacts/nmpc_build/aarch64/nmpc_solver"
            build_solver.mkdir()
            (build_solver / "libnmpc_bundle_aarch64.a").write_bytes(b"linux arm")

    monkeypatch.setattr(gen_nmpc_lib.shutil, "which", lambda _name: "/usr/bin/docker")
    monkeypatch.setattr(gen_nmpc_lib, "_host_arch", lambda: "x86_64")
    monkeypatch.setattr(gen_nmpc_lib.subprocess, "run", run)
    gen_nmpc_lib._docker_build_bundle(tmp_path, "aarch64")
    command = calls[-1]
    assert f"{tmp_path}:/ws:ro" in command
    assert f"{tmp_path}/.artifacts/nmpc_build/aarch64:/ws/.artifacts:rw" in command
    assert "--ipc=host" in command
    assert (solver / "libnmpc_bundle_aarch64.a").read_bytes() == b"linux arm"
    assert alias.read_bytes() == b"host x86"
    assert not list(solver.glob("tmp*.a"))


def test_simple_build_uses_same_targeting(monkeypatch):
    compose, bundle = Mock(), Mock()
    monkeypatch.setattr(sim, "compose", compose)
    monkeypatch.setattr(sim, "ensure_bundle", bundle)
    sim.build(target="simple", arch="arm64")
    bundle.assert_called_once_with(ROOT, "arm64")
    assert compose.call_args.kwargs["arch"] == "arm64"
    with pytest.raises(typer.BadParameter):
        sim.build(target="typo")


def test_full_install_does_not_install_host_linux_toolchain(monkeypatch):
    build = Mock()
    monkeypatch.setattr(docker, "build", build)
    stack.full_install(docker_all=True, arch="arm64")
    build.assert_called_once_with(services=None, arch="arm64", gui=True)


@pytest.mark.parametrize(
    "name",
    [
        "infra/sim_infra/gazebo/docker/docker-compose.yml",
        "infra/sim_infra/simple_sim/docker/docker-compose.yml",
        "core/docker/docker-compose.yml",
    ],
)
def test_every_service_uses_host_ipc(name):
    config = yaml.safe_load((ROOT / name).read_text())
    assert all(service["ipc"] == "host" for service in config["services"].values())


def test_headless_defaults_and_loopback_only_desktop():
    config = yaml.safe_load((ROOT / "infra/sim_infra/gazebo/docker/docker-compose.yml").read_text())
    services = config["services"]
    assert services["ros2-base"]["profiles"] == ["build"]
    for name in docker.GUI_SERVICES:
        assert services[name]["profiles"] == ["gui"]
    for service in services.values():
        assert "runtime" not in service
        assert not service.get("privileged", False)
    assert services["desktop"]["ports"] == ["127.0.0.1:6088:6080"]
    assert "qground_control" not in services["ap-pnc"]["depends_on"]


@pytest.mark.skipif(shutil.which("docker") is None, reason="Docker Compose CLI not installed")
@pytest.mark.parametrize("arch", ["arm64", "amd64"])
@pytest.mark.parametrize("gui", [False, True])
def test_compose_runtime_profiles_resolve_without_build_service(arch, gui):
    # Real parser regression: service: build contexts referencing a disabled
    # build-only profile can pass all-profile config but break normal `up`.
    file = ROOT / "infra/sim_infra/gazebo/docker/docker-compose.yml"
    command = ["docker", "compose", "-f", str(file)]
    if gui:
        command += ["--profile", "gui"]
    output = subprocess.run(
        [*command, "config", "--format", "json"],
        env=runtime.compose_env(ROOT, arch),
        capture_output=True,
        text=True,
        check=True,
    )
    services = json.loads(output.stdout)["services"]
    assert "ros2-base" not in services
    assert ("desktop" in services) is gui
    assert all(service["platform"] == f"linux/{arch}" for service in services.values())
    assert all(service["ipc"] == "host" for service in services.values())


@pytest.mark.skipif(shutil.which("docker") is None, reason="Docker Compose CLI not installed")
@pytest.mark.parametrize("arch", ["arm64", "amd64"])
def test_simple_runtime_profile_resolves(arch):
    file = ROOT / "infra/sim_infra/simple_sim/docker/docker-compose.yml"
    output = subprocess.run(
        ["docker", "compose", "-f", str(file), "config", "--format", "json"],
        env=runtime.compose_env(ROOT, arch),
        capture_output=True,
        text=True,
        check=True,
    )
    services = json.loads(output.stdout)["services"]
    assert set(services) == {"simple-sim"}
    assert services["simple-sim"]["image"] == f"simple-sim:{arch}"
    assert services["simple-sim"]["ipc"] == "host"
