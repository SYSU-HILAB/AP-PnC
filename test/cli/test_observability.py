"""Telemetry contracts; run with uv run --extra telemetry pytest ..."""

import json
import sys
from pathlib import Path

import pytest

from tooling import observability as otel


@pytest.fixture
def telemetry(monkeypatch):
    pytest.importorskip("opentelemetry.sdk")
    from opentelemetry.sdk._logs.export import InMemoryLogRecordExporter
    from opentelemetry.sdk.metrics.export import InMemoryMetricReader
    from opentelemetry.sdk.trace.export.in_memory_span_exporter import InMemorySpanExporter

    monkeypatch.setattr(otel, "_settings", otel.TelemetrySettings())
    monkeypatch.delenv("TRACEPARENT", raising=False)
    monkeypatch.delenv("TRACESTATE", raising=False)
    monkeypatch.setenv("OTEL_EXPORTER_OTLP_ENDPOINT", "http://127.0.0.1:4318")
    spans, logs, metrics = (
        InMemorySpanExporter(),
        InMemoryLogRecordExporter(),
        InMemoryMetricReader(),
    )
    instance = otel.Telemetry(
        trace_exporter=spans, log_exporter=logs, metric_reader=metrics, sample_ratio=1
    )
    yield instance, spans, logs, metrics
    instance.close()


def test_disabled_path_does_not_import_sdk(monkeypatch):
    monkeypatch.setattr(otel, "_settings", otel.TelemetrySettings(enabled=False))
    before = set(sys.modules)
    with otel.session(), otel.operation("private raw name deliberately not inspected") as span:
        assert span is None and otel.carrier() == {}
        otel.event("private event deliberately not inspected")
    assert not any(k.startswith("opentelemetry") for k in set(sys.modules) - before)


def test_attributes_fail_closed():
    result = otel.attributes(
        {
            "password": "secret",
            "authorization": "secret",
            "path": "/private/source",
            "ap.pnc.tick": True,
            "ap.pnc.model.aero": "secret model",
            "ap.pnc.sim_time_ns": 123,
            "ap.pnc.frame.body": "FLU",
            "ap.pnc.experiment.id": "bad",
            "ap.pnc.artifact.sha256": "a" * 64,
        }
    )
    assert result == {
        "ap.pnc.sim_time_ns": 123,
        "ap.pnc.frame.body": "FLU",
        "ap.pnc.artifact.sha256": "a" * 64,
    }


@pytest.mark.parametrize(
    "url",
    [
        "http://public.example:4318",
        "http://user:secret@localhost:4318",
        "http://localhost:4318/?token=secret",
        "http://localhost:4318/v1/traces",
        "file:///private/path",
        "http://localhost:100000",
    ],
)
def test_endpoint_blocks_unapproved_destinations(url, monkeypatch):
    monkeypatch.setenv("OTEL_EXPORTER_OTLP_ENDPOINT", url)
    monkeypatch.setattr(otel, "_settings", otel.TelemetrySettings(allow_remote=False))
    with pytest.raises(ValueError):
        otel.endpoint()


def test_three_signals_correlate_and_do_not_leak(telemetry):
    current, spans, logs, metrics = telemetry
    with otel.session(telemetry=current):
        with otel.operation(
            "experiment",
            {"path": "PRIVATE SOURCE", "ap.pnc.tick": 100, "ap.pnc.sim_time_ns": 2000000000},
        ):
            with pytest.raises(RuntimeError):
                with otel.operation("nmpc.solve", {"ap.pnc.solver.status": 4}):
                    raise RuntimeError("PRIVATE ERROR BODY")
    data = metrics.get_metrics_data()  # Collect exemplars once, before force_flush drains them.
    assert current.flush()
    completed = spans.get_finished_spans()
    assert len(completed) == 2
    child, parent = completed
    assert (
        child.context.trace_id == parent.context.trace_id
        and child.parent.span_id == parent.context.span_id
    )
    assert (
        parent.attributes["ap.pnc.result"] == "ok" and child.attributes["ap.pnc.result"] == "error"
    )
    assert not child.events  # no implicit exception stack/arguments
    assert "PRIVATE" not in str(completed) and "path" not in parent.attributes
    emitted = logs.get_finished_logs()
    assert len(emitted) == 5
    assert all(x.log_record.trace_id == parent.context.trace_id for x in emitted)
    assert all("PRIVATE" not in str(x.log_record) for x in emitted)
    points = [
        p
        for r in data.resource_metrics
        for s in r.scope_metrics
        for m in s.metrics
        for p in m.data.data_points
    ]
    assert points and all(
        set(p.attributes) == {"ap.pnc.operation", "ap.pnc.result"} for p in points
    )
    assert any(p.exemplars for p in points)


def test_success_exit_not_failure(telemetry):
    current, spans, _, _ = telemetry
    with otel.session(telemetry=current), pytest.raises(SystemExit):
        with otel.operation("cli.command"):
            raise SystemExit(0)
    current.flush()
    assert spans.get_finished_spans()[0].attributes["ap.pnc.result"] == "ok"


def test_no_global_provider_or_resource_env_mutation(telemetry, monkeypatch):
    from opentelemetry import trace

    current, spans, _, _ = telemetry
    original = trace.get_tracer_provider()
    with otel.session(telemetry=current), otel.operation("experiment"):
        pass
    current.flush()
    assert trace.get_tracer_provider() is original
    assert "host.name" not in spans.get_finished_spans()[0].resource.attributes


def test_w3c_parent_propagation(telemetry, monkeypatch):
    current, spans, _, _ = telemetry
    with otel.session(telemetry=current), otel.operation("experiment") as parent:
        saved = otel.carrier()
        assert "baggage" not in saved
        parent_id = parent.get_span_context().span_id
    monkeypatch.setenv("TRACEPARENT", saved["traceparent"])
    with otel.session(telemetry=current), otel.operation("dora.node"):
        pass
    current.flush()
    assert spans.get_finished_spans()[1].parent.span_id == parent_id


def test_nested_session_lifecycle(telemetry):
    current, *_ = telemetry
    with otel.session(telemetry=current):
        with otel.session() as nested:
            assert nested is current
    assert otel._session.get() is None


def test_collector_command_is_scoped_and_paths_absolute(monkeypatch, tmp_path):
    from tooling import docker_runtime, observability as cli

    monkeypatch.setenv("AP_PNC_DIR", str(tmp_path))
    compose_file = tmp_path / "infra/observability/docker-compose.yaml"
    compose_file.parent.mkdir(parents=True)
    compose_file.write_text(
        "services:\n  collector:\n    image: otel/opentelemetry-collector:arm64\n"
        "    u,ser: \"1000:1000\"\n".replace("u,ser", "user")
    )
    calls = []
    monkeypatch.setattr(
        docker_runtime.subprocess,
        "run",
        lambda command, **kwargs: calls.append((command, kwargs)),
    )
    cli.collector(["up", "-d"], arch="amd64")
    command, kwargs = calls[0]
    assert command[3] == str(compose_file)
    assert "AP_PNC_ARCH" not in kwargs["env"]
    assert kwargs["env"]["AP_PNC_DIR"] == str(tmp_path) and kwargs["check"]
    override = Path(command[command.index("-f") + 3])
    assert ":amd64" in override.read_text()
    assert (tmp_path / ".artifacts/telemetry/local").is_dir()
    assert "PX4" not in json.dumps(command)
