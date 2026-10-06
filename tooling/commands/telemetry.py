"""Local OTel Collector lifecycle and an explicit three-signal smoke probe."""

import json
import os
import subprocess

import typer

from tooling.docker_runtime import compose_env
from tooling.env import artifact_path, get_git_root
from tooling.observability import Telemetry, carrier, operation, session

app = typer.Typer(no_args_is_help=True)


def collector(args, arch="native"):
    root = get_git_root()
    artifact_path(root / ".artifacts/telemetry/local").mkdir(
        parents=True, exist_ok=True, mode=0o700
    )
    subprocess.run(
        ["docker", "compose", "-f", str(root / "infra/observability/docker-compose.yaml"), *args],
        env={
            **compose_env(root, arch),
            "AP_PNC_UID": str(os.getuid()),
            "AP_PNC_GID": str(os.getgid()),
        },
        check=True,
    )


@app.command()
def up(arch: str = typer.Option("native", "--arch")):
    """Start only the local Collector, never PX4/ROS or a remote backend."""
    collector(["up", "-d"], arch)


@app.command()
def down():
    """Stop only this Compose project; retain local telemetry evidence."""
    collector(["down"])


@app.command()
def status():
    collector(["ps"])


@app.command()
def smoke():
    """Send known success/error spans, metrics and correlated logs over OTLP/HTTP."""
    telemetry = Telemetry("ap-pnc-smoke", sample_ratio=1.0)
    try:
        with session(telemetry=telemetry):
            with operation("telemetry.smoke", {"ap.pnc.sim_time_ns": 0}) as span:
                with operation("config.load"):
                    pass
                try:
                    with operation("nmpc.solve", {"ap.pnc.solver.status": 4, "ap.pnc.tick": 0}):
                        raise RuntimeError("probe-only private error body MUST NOT be exported")
                except RuntimeError:
                    pass
                typer.echo(
                    json.dumps(
                        {
                            "trace_id": format(span.get_span_context().trace_id, "032x"),
                            "carrier": carrier(),
                            "mode": "synthetic collector probe, not flight/solver acceptance",
                        }
                    )
                )
        telemetry.flush()
    finally:
        telemetry.close()
