"""Opt-in OTel foundation. No global provider replacement, raw arguments or file bodies.

Providers belong to a session; ContextVar isolates nested/concurrent callers. The SDK
is imported only when enabled. Domain callers use operation()/event()/carrier().
"""

import os
import re
import time
import uuid
import warnings
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from importlib.metadata import version
from urllib.parse import urlparse

from tooling.env import artifact_path, get_git_root

OPERATIONS = frozenset(
    {
        "cli.command",
        "experiment",
        "dora.node",
        "planner.optimize",
        "nmpc.solve",
        "plant.advance",
        "motor.prepare",
        "aero.evaluate",
        "replay.export",
        "replay.verify",
        "telemetry.smoke",
        "config.load",
        "artifact.write",
    }
)
ENUMS = {
    "ap.pnc.operation": OPERATIONS,
    "ap.pnc.result": {"ok", "error", "cancelled"},
    "ap.pnc.arch": {"arm64", "amd64"},
    "ap.pnc.model.aero": {"none", "lyu", "phi", "ma", "advanced"},
    "ap.pnc.model.actuator": {"ideal_rate_loop", "practical"},
    "ap.pnc.frame.world": {"ENU"},
    "ap.pnc.frame.body": {"FLU"},
    "ap.pnc.error.type": {
        "RuntimeError",
        "ValueError",
        "TimeoutError",
        "FileNotFoundError",
        "OSError",
        "CalledProcessError",
        "KeyboardInterrupt",
        "SystemExit",
        "other",
    },
}
INTEGER_KEYS = {
    "ap.pnc.sim_time_ns",
    "ap.pnc.tick",
    "ap.pnc.stage",
    "ap.pnc.horizon.count",
    "ap.pnc.solver.status",
    "ap.pnc.nmpc.abi",
}
HASH_KEYS = {"ap.pnc.artifact.sha256", "ap.pnc.reference.sha256", "ap.pnc.solver.bundle.sha256"}
_session = ContextVar("ap_pnc_telemetry_session", default=None)


def attributes(values):
    """Fail-closed allowlist. Unknown keys and raw/private strings never leave here."""
    out = {}
    for key, value in (values or {}).items():
        if key in ENUMS and isinstance(value, str) and value in ENUMS[key]:
            out[key] = value
        elif key in INTEGER_KEYS and type(value) is int and abs(value) < 2**63:
            out[key] = value
        elif key in HASH_KEYS and isinstance(value, str) and re.fullmatch("[a-f0-9]{64}", value):
            out[key] = value
        elif (
            key == "ap.pnc.command"
            and isinstance(value, str)
            and re.fullmatch("[a-z][a-z0-9-]{0,31}", value)
        ):
            out[key] = value
        elif key == "ap.pnc.experiment.id" and isinstance(value, str):
            try:
                out[key] = str(uuid.UUID(value))
            except ValueError:
                pass
    return out


@dataclass(frozen=True)
class TelemetrySettings:
    """Where the tooling's telemetry goes.

    The module owns the defaults and callers override them explicitly: this is a
    parameter of the tooling, not an environment variable. `AP_PNC_DIR` remains
    the only environment variable the project defines.
    """

    enabled: bool = False
    allow_remote: bool = False
    sample_ratio: float = 1.0
    experiment_id: str | None = None


_settings = TelemetrySettings()


def configure(settings: TelemetrySettings) -> None:
    """Install the telemetry settings for this process."""
    global _settings
    _settings = settings


def endpoint():
    base = os.getenv("OTEL_EXPORTER_OTLP_ENDPOINT", "http://127.0.0.1:4318").rstrip("/")
    url = urlparse(base)
    _ = url.port  # Reject invalid/out-of-range ports before initializing exporters.
    if (
        url.scheme not in ("http", "https")
        or not url.hostname
        or url.username
        or url.password
        or url.query
        or url.fragment
        or url.path not in ("", "/")
    ):
        raise ValueError("OTLP endpoint must be an HTTP(S) base URL without credentials/path/query")
    if url.hostname not in {"127.0.0.1", "localhost", "::1", "otel-collector"}:
        if not _settings.allow_remote or url.scheme != "https":
            raise ValueError("remote telemetry requires explicit opt-in and HTTPS")
    return base


class Telemetry:
    """Bounded background exporters; optional in-memory readers/exporters for tests."""

    def __init__(
        self,
        service="ap-pnc-cli",
        *,
        trace_exporter=None,
        metric_reader=None,
        log_exporter=None,
        sample_ratio=None,
    ):
        from opentelemetry.exporter.otlp.proto.http._log_exporter import OTLPLogExporter
        from opentelemetry.exporter.otlp.proto.http.metric_exporter import OTLPMetricExporter
        from opentelemetry.exporter.otlp.proto.http.trace_exporter import OTLPSpanExporter
        from opentelemetry.sdk._logs import LoggerProvider
        from opentelemetry.sdk._logs.export import BatchLogRecordProcessor
        from opentelemetry.sdk.metrics import MeterProvider
        from opentelemetry.sdk.metrics.export import PeriodicExportingMetricReader
        from opentelemetry.sdk.resources import Resource
        from opentelemetry.sdk.trace import TracerProvider
        from opentelemetry.sdk.trace.export import BatchSpanProcessor
        from opentelemetry.sdk.trace.sampling import ParentBased, TraceIdRatioBased

        ratio = _settings.sample_ratio if sample_ratio is None else sample_ratio
        if not 0 <= ratio <= 1:
            raise ValueError("trace sample ratio must be finite and in [0,1]")
        base = endpoint()
        self.id = (
            str(uuid.UUID(_settings.experiment_id))
            if _settings.experiment_id
            else str(uuid.uuid4())
        )
        if service not in {"ap-pnc-cli", "ap-pnc-smoke"}:
            raise ValueError("unregistered telemetry service")
        resource = Resource(
            {
                "service.name": service,
                "service.version": version("ap-pnc"),
                "service.instance.id": str(uuid.uuid4()),
                "deployment.environment.name": "development",
                "telemetry.sdk.name": "opentelemetry",
                "telemetry.sdk.language": "python",
                "telemetry.sdk.version": version("opentelemetry-sdk"),
            }
        )
        self.traces = TracerProvider(
            resource=resource, sampler=ParentBased(TraceIdRatioBased(ratio)), shutdown_on_exit=False
        )
        self.traces.add_span_processor(
            BatchSpanProcessor(
                trace_exporter or OTLPSpanExporter(endpoint=base + "/v1/traces", timeout=1),
                max_queue_size=2048,
                max_export_batch_size=256,
                schedule_delay_millis=1000,
                export_timeout_millis=1500,
            )
        )
        reader = metric_reader or PeriodicExportingMetricReader(
            OTLPMetricExporter(endpoint=base + "/v1/metrics", timeout=1),
            export_interval_millis=10000,
            export_timeout_millis=1500,
        )
        self.metrics = MeterProvider(
            resource=resource, metric_readers=[reader], shutdown_on_exit=False
        )
        self.logs = LoggerProvider(resource=resource, shutdown_on_exit=False)
        self.logs.add_log_record_processor(
            BatchLogRecordProcessor(
                log_exporter or OTLPLogExporter(endpoint=base + "/v1/logs", timeout=1),
                max_queue_size=2048,
                max_export_batch_size=256,
                schedule_delay_millis=1000,
                export_timeout_millis=1500,
            )
        )
        self.tracer = self.traces.get_tracer("ap-pnc", version("ap-pnc"))
        self.logger = self.logs.get_logger("ap-pnc", version("ap-pnc"))
        meter = self.metrics.get_meter("ap-pnc", version("ap-pnc"))
        self.calls = meter.create_counter("ap.pnc.operation.calls", unit="{operation}")
        self.duration = meter.create_histogram(
            "ap.pnc.operation.duration",
            unit="s",
            explicit_bucket_boundaries_advisory=(
                0.0001,
                0.0005,
                0.001,
                0.002,
                0.005,
                0.01,
                0.02,
                0.05,
                0.1,
                0.5,
                1,
                5,
                30,
            ),
        )

    def flush(self):
        return all(
            [
                self.traces.force_flush(timeout_millis=1500),
                self.logs.force_flush(timeout_millis=1500),
                self.metrics.force_flush(timeout_millis=1500),
            ]
        )

    def close(self):
        # Shutdown only these providers, never a host application's global providers.
        for provider in (self.traces, self.logs, self.metrics):
            try:
                provider.shutdown()
            except Exception:
                warnings.warn(
                    "OTel shutdown failed; scientific/application result is unchanged",
                    RuntimeWarning,
                    stacklevel=2,
                )


@contextmanager
def session(service="ap-pnc-cli", *, telemetry=None):
    existing = _session.get()
    if existing is not None:
        yield existing
        return
    enabled = _settings.enabled
    if telemetry is None and not enabled:
        yield None
        return
    owned = telemetry is None
    current = telemetry or Telemetry(service)
    token = _session.set(current)
    try:
        yield current
    finally:
        _session.reset(token)
        if owned:
            current.close()


@contextmanager
def operation(name, values=None):
    current = _session.get()
    if current is None:
        yield None
        return
    if name not in OPERATIONS:
        raise ValueError("unregistered telemetry operation")
    from opentelemetry import trace
    from opentelemetry.trace.propagation.tracecontext import TraceContextTextMapPropagator

    safe = attributes(values)
    safe.update({"ap.pnc.operation": name, "ap.pnc.experiment.id": current.id})
    parent = None
    if not trace.get_current_span().get_span_context().is_valid:
        # Explicit W3C trace context only: never import arbitrary baggage/argv.
        parent = TraceContextTextMapPropagator().extract(
            {"traceparent": os.getenv("TRACEPARENT", ""), "tracestate": os.getenv("TRACESTATE", "")}
        )
    start = time.perf_counter()
    result = "ok"
    with current.tracer.start_as_current_span(
        name, context=parent, attributes=safe, record_exception=False, set_status_on_exception=False
    ) as span:
        event("operation.started", safe)
        try:
            yield span
        except BaseException as error:
            if isinstance(error, SystemExit) and error.code in (None, 0):
                raise
            result = "cancelled" if isinstance(error, KeyboardInterrupt) else "error"
            kind = type(error).__name__
            if kind not in ENUMS["ap.pnc.error.type"]:
                kind = "other"
            span.set_attribute("ap.pnc.error.type", kind)
            span.set_status(trace.Status(trace.StatusCode.ERROR))
            event("operation.failed", {**safe, "ap.pnc.error.type": kind, "ap.pnc.result": result})
            raise
        finally:
            span.set_attribute("ap.pnc.result", result)
            labels = {"ap.pnc.operation": name, "ap.pnc.result": result}
            # NO run/tick/trace-id/hash labels in time series.
            current.calls.add(1, labels)
            current.duration.record(time.perf_counter() - start, labels)
            event("operation.finished", {**safe, "ap.pnc.result": result})


def event(name, values=None):
    current = _session.get()
    if current is None:
        return
    if name not in {"operation.started", "operation.finished", "operation.failed"}:
        raise ValueError("unregistered telemetry event")
    from opentelemetry._logs import SeverityNumber

    current.logger.emit(
        body=name,
        event_name=name,
        severity_number=SeverityNumber.ERROR if name == "operation.failed" else SeverityNumber.INFO,
        attributes=attributes(values),
    )


def carrier():
    """W3C carrier for a child process/message. Baggage intentionally unsupported."""
    if _session.get() is None:
        return {}
    from opentelemetry.trace.propagation.tracecontext import TraceContextTextMapPropagator

    out = {}
    TraceContextTextMapPropagator().inject(out)
    return out


def collector(args: list[str], arch: str = "native") -> None:
    """Run the local OpenTelemetry collector compose file.

    Goes through the Compose layer in :mod:`tooling.docker_runtime`, so the arch and
    the container user come from its generated override rather than the
    environment.
    """
    from tooling import docker_runtime  # - keeps the import graph flat

    root = get_git_root()
    artifact_path(root / ".artifacts/telemetry/local").mkdir(
        parents=True, exist_ok=True, mode=0o700
    )
    docker_runtime.compose(
        root, root / "infra/observability/docker-compose.yaml", args, arch=arch
    )
