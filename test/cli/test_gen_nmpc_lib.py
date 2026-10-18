"""Docker-only codegen must not fall back to host tools or corrupt target bundles."""

import os
import runpy
import subprocess
from pathlib import Path
from unittest.mock import Mock

import pytest
from typer.testing import CliRunner

from tooling.commands import gen_nmpc_lib as gen
from tooling.env import get_git_root

ROOT = get_git_root()


@pytest.mark.parametrize("host", ["aarch64", "x86_64"])
@pytest.mark.parametrize("arch", ["native", "aarch64", "x86_64"])
def test_cli_only_uses_docker(tmp_path, monkeypatch, host, arch):
    target = host if arch == "native" else arch
    solver = tmp_path / ".artifacts/nmpc_solver"
    solver.mkdir(parents=True)
    alias = solver / "libnmpc_bundle.a"
    alias.write_bytes(b"existing host alias")
    tagged = solver / f"libnmpc_bundle_{target}.a"

    def build(root, selected):
        assert (root, selected) == (tmp_path, target)
        tagged.write_bytes(b"docker target archive")

    builder = Mock(side_effect=build)
    native_command = Mock(side_effect=AssertionError("host toolchain must not run"))
    monkeypatch.setattr(gen, "get_git_root", lambda: tmp_path)
    monkeypatch.setattr(gen, "_host_arch", lambda: host)
    monkeypatch.setattr(gen, "_docker_build_bundle", builder)
    monkeypatch.setattr(gen.subprocess, "run", native_command)
    result = CliRunner().invoke(gen.app, ["--arch", arch])
    assert result.exit_code == 0, result.output
    builder.assert_called_once_with(tmp_path, target)
    native_command.assert_not_called()
    assert tagged.read_bytes() == b"docker target archive"
    assert alias.read_bytes() == b"existing host alias"


def test_bad_arch_never_starts_builder(monkeypatch):
    builder = Mock()
    monkeypatch.setattr(gen, "_docker_build_bundle", builder)
    result = CliRunner().invoke(gen.app, ["--arch", "riscv64"])
    assert result.exit_code == 2
    builder.assert_not_called()


def test_native_docker_build_only_publishes_tag(tmp_path, monkeypatch):
    commands = []

    def run(args, **kwargs):
        commands.append(args)
        if args[:2] == ["docker", "run"]:
            solver = tmp_path / ".artifacts/nmpc_build/aarch64/nmpc_solver"
            solver.mkdir()
            (solver / "libnmpc_bundle_aarch64.a").write_bytes(b"linux arm64")

    monkeypatch.setattr(gen.shutil, "which", lambda _name: "/usr/bin/docker")
    monkeypatch.setattr(gen, "_host_arch", lambda: "aarch64")
    monkeypatch.setattr(gen.subprocess, "run", run)
    gen._docker_build_bundle(tmp_path, "aarch64")
    assert [command[1] for command in commands] == ["build", "run"]
    assert all(command[command.index("--platform") + 1] == "linux/arm64" for command in commands)
    solver = tmp_path / ".artifacts/nmpc_solver"
    assert (solver / "libnmpc_bundle_aarch64.a").read_bytes() == b"linux arm64"
    assert not (solver / "libnmpc_bundle.a").exists()


def test_entrypoint_passes_image_toolchain_paths(tmp_path, monkeypatch):
    # Run the real shell entrypoint, intercepting only Python/codegen. Then execute
    # its captured Python block against a mock to verify the wrapper's arguments.
    shim = tmp_path / "python-shim.sh"
    shim.write_text(
        'uname() { printf aarch64; }\n'
        'python3() {\n'
        '  if [[ "$1" == "-c" ]]; then\n'
        '    printf "%s" "$2" > "$AP_PNC_DIR/wrapper.py"\n'
        '  fi\n'
        '}\n'
    )
    subprocess.run(
        ["bash", str(ROOT / "core/docker/nmpc-builder-entrypoint.sh")],
        env={
            **os.environ,
            "AP_PNC_DIR": str(tmp_path),
            "BASH_ENV": str(shim),
        },
        check=True,
        capture_output=True,
        text=True,
    )
    build = Mock()
    monkeypatch.setattr(gen, "_build_wrapper_and_bundle", build)
    monkeypatch.setenv("AP_PNC_DIR", str(tmp_path))
    runpy.run_path(str(tmp_path / "wrapper.py"))
    build.assert_called_once_with(
        tmp_path, arch="aarch64",
        staging_root=Path("/opt/acados"), static_dir=Path("/opt/acados_static")
    )
