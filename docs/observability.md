# Project-wide OpenTelemetry

## Decision and current acceptance boundary

OpenTelemetry is the project's operational observability contract, not an optional
collection of uncorrelated print statements. Adopt traces, bounded-cardinality
metrics and structured correlated events across the Rust-first architecture.
Scientific CSV/YAML/reference/metrics/manifest and independently verified Rerun
recordings remain the numerical authority.

**Foundation implemented, full coverage incomplete:** Python CLI session tracing,
three-signal OTLP/HTTP export, W3C trace-parent helpers, explicit attribute filtering,
and a native multiarch, loopback-only, file-backed development Collector. Python
SDK/exporter are pinned to 1.45.0; Collector 0.162.0 is pinned by manifest digest.
Rust events currently use dora's structured log pipeline; Rust OTLP log forwarding
has not yet been added. Do not conflate the Python three-signal smoke with it.
**Dora graph integration implemented and verified on native Linux ARM64:** real
simulate → record → verify share a trace through the reserved dora metadata key;
C++ subprocess and independent decoder boundaries are observed. Actual logged
solver status/tick/simulation time/reference hash are linked to the subprocess span,
and measured solve durations are exported as log-derived histograms. C++ numerical
function spans, ROS/SITL telemetry, dashboards, production storage/authentication,
tail sampling and profiling remain subsequent gates. A synthetic Python
`nmpc.solve` smoke span is **not** a real acados solver measurement.

**Reuse dora's OTel design first; do not build a parallel Rust telemetry runtime.**
Upstream dora 1.0.1 (`c2c6f70a06444add9dca68859b0db1fc831fde2c`) already has
Rust tracing/OTLP, metric export and reserved metadata trace propagation. The
AP-PnC node dependency now explicitly enables upstream `tracing` and `metrics`
while leaving unrelated default features off. Its public `init_tracing` owns the
subscriber/OTLP providers. Node scopes attach their exact OTel context before
send_output; consumers use the reserved metadata key. There is no competing
Rust provider or new trace envelope.

Dora node logs and Rerun's transitive OTel crates alone are not full numerical
instrumentation. Python CLI's separate-language SDK and the shared Collector
remain useful; they must join dora's trace context, not create a competing root.
Do not replace another library's global provider/subscriber.

## Coverage contract (target, not completed instrumentation)

| Layer | Required spans/events | Required metrics and diagnostics |
|---|---|---|
| CLI / builder | command, dependency/codegen/build/link stages, process exit/cancellation | operation duration, result; artifact/binary/solver hashes on spans |
| Python API / data / ML | HTTP route/task lifecycle, dataset/profile import, training/evaluation/codegen/FFI validation | route latency/error, job backlog, stage/epoch duration, numerical validation failures; no bodies/query/weights |
| Rust dora engine | experiment root, node startup, input/output/queue wait, simulate → record → verify | message age, queue occupancy, restarts, loss, export backlog |
| Planner | frontend, corridor, MINCO objective, line search, FLU value/J/VJP, reference save | iterations, cost components, bound violation, fallback/branch counts |
| NMPC | reference assembly, x0/parameter submission, prepare/feedback/solve, output commit | status, QP iterations/residuals, solve latency, deadlines, warm-start/reset counts |
| Plant / aero | control period, actuator preparation, aggregate integration, aero model call/failure | period duration, nonfinite/reject counts, torque/force aggregates, RPM/duty/limit/PID counters |
| ROS / PX4 / navigation | callbacks, sample selection, estimator/LIO and setpoint boundaries, offboard/arm/safety transitions | input age, jitter, missed deadlines, QoS/transport loss, heartbeat age; external black-box limits |
| Artifacts / replay | atomic write, hash check, export, independent decode, acceptance | rows, bytes, integrity failures, completion/failure counts |
| Collector / host | receiver/exporter/queue health, process/VM resources | dropped telemetry, memory, queue fill, retry/failure, disk/rotation |

Input/reference/actuator model selection, solver ABI, source revision, binary/bundle
hashes, frame contract and clock policy accompany the experiment. Detailed values
are referenced by local artifact digest + tick/stage, not copied into every span.
No automatic arming, flight, PX4 rebuild, privileged profiling or remote export is
part of observability setup.

## Trace and causal model

One `experiment` root covers a run, with logical subspans for planning, simulation,
export and independent verification. A controller period can own a sampled
`nmpc.solve` span; integration and actuator work are children/aggregated events.
Reuse dora's public `OPEN_TELEMETRY_CONTEXT` / `Metadata::open_telemetry_context`
at Arrow/message boundaries. Its tracing-enabled send path injects the ambient
OTel context when this metadata key is absent; explicit context wins. The upstream
codec carries W3C traceparent/tracestate, not baggage. Configure
`DORA_OTLP_ENDPOINT` for its gRPC exporter (normally port 4317); Python's HTTP
exporter uses port 4318 on the same Collector. A compiled feature/endpoint is not
proof that every node initialized its subscriber and established a valid context.

- A single input operation may be the parent of its consumer.
- Fan-in, delayed sensor samples and reused references use **span links** to their
  producers. Never invent synchronous parentage for asynchronous DDS/Arrow traffic.
- Maintain experiment UUID across actors; add source tick/sequence and artifact
  association alongside dora's existing metadata, without replacing path payloads
  or inventing a second trace codec. Reject malformed context without corrupting
  scientific payloads.
- Put context/trace indices in a separate observability sidecar, not into an already
  frozen scientific schema/hash. Failed/interrupted runs retain their failure state.
- External PX4/MAVROS/vendor SDKs can be black-box boundaries: ingress/egress and
  message age are measurable; internal spans require genuine instrumentation.

The Python `operation()` helper can extract explicitly supplied `TRACEPARENT`.
The real dora graph now propagates through `OPEN_TELEMETRY_CONTEXT`; its dataflow
UUID identifies the experiment across actors. C++ subprocess invocation receives
TRACEPARENT and experiment ID. The current C++ executable does not initialize an
OTel SDK from those headers: its measured process boundary and returned scientific
diagnostics are observed by Rust, not automatically instrumented C++ functions.
CLI-to-dora launch-parent wiring remains separate from the verified graph chain.

### C++ bridge: follow the upstream boundary, not a second telemetry engine

Upstream `apis/c++/node/src/lib.rs` uses `#[cxx::bridge]`, with a Rust `staticlib`
behind the C++ node API. C++ calls Rust `dora-node-api`; dora is not implemented
on top of a C++ OTel SDK. The C++ crate's default `tracing` feature enables its
Rust node dependency's tracing feature, and typed metadata supports passing the
same context. It does not automatically create spans inside arbitrary C++
`acados_solve`, RK4, aero or PID calls. Subscriber startup/parent attachment must
be tested, not inferred from linked feature flags.

Our Rust simulator invokes a separate C++ executable (`Command`), **not**
this dora CXX API. The current integration reuses dora's lifecycle and instruments
that process boundary; it also passes context to the process. Next: add minimal
in-process hooks/RAII span handles for actual numerical stage boundaries if the
engine is linked through CXX/FFI, or consume the context in the standalone process.
Do not turn every derivative/solver function into a dora message or remote call.
Keep the numerical period local and preserve Rust-first migration.

Prefer public `dora-node-api` initialization/metadata APIs. Upstream
`libraries/extensions/telemetry/tracing` explicitly states it is internal and
outside the 1.0 stability guarantee; do not depend on that crate casually.

## Clocks: never rewrite time to make a trace look scientific

- OTel start/end timestamps are Unix wall-clock nanoseconds. Duration uses a
  monotonic clock. They measure execution cost, not the simulated flight duration.
- `ap.pnc.sim_time_ns`, tick and stage are explicit span/event attributes.
- Rerun plots and scientific data retain simulation time and committed state rules.
- Host/VM/remote clock offsets are measured; do not compare raw unsynchronized
  cross-host timestamps as queue delays. Also record monotonic receive/send ages.
- Prediction knots use their **future reference time**; they are not current plant
  measurements. Body FLU, world ENU and legacy model boundaries stay explicit.

## Instrumentation depth and real-time limits

Use three modes: off, normal operational coverage and explicitly enabled diagnostic
coverage. Normal mode retains run/lifecycle/failure evidence and aggregate metrics;
it does not create a span for every RK4 derivative at 1 kHz. Diagnostic spans can
sample control periods and add planner/QP detail. Profiling is a separate,
permissioned diagnostic mode; OTel profiling support is evolving and is not a
stable, universally available fourth signal.

Hot C++/Rust paths must not do network/file I/O, force-flush, format raw messages,
allocate large attributes or block on exporter locks. Use bounded background queues
and aggregate/atomic counters. Expose queue drops and exporter health. A telemetry
outage never changes controller commands, commits a failed integration period or
turns a failed experiment into success.

Python foundation uses background batches (2048 records, 256 per export), one-second
HTTP exporter timeout and explicit lifecycle shutdown. Defaults are **off** until
per-layer overhead is measured. Python shutdown is not a real-time operation; it
must remain outside the control period. SDK queue limits do not establish a measured
hard real-time bound or guarantee that every failed sampled-out span is retained.

Head sampling honors parent context. Failure retention later needs a bounded local
flight recorder plus Collector tail sampling: a head-dropped trace cannot be
reconstructed by tail sampling. Size that pipeline from expected rate, decision
window and memory. Unsampled failure counts still need aggregate metrics.

## Metrics, logs and privacy

- Instruments use explicit units: seconds, meters, radians, N/kg, Nm, RPM and bytes.
- Histogram buckets include the 1 ms integration and 20 ms control-period ranges;
  deadlines are verified against actual scheduled wall-clock work, not sim time.
- Metric labels are bounded enums (component, operation, result/model/status class).
  No run UUID, tick, trace ID, path, hash, quaternion or vector component value as a
  time-series label. Exemplars link aggregate metrics to traces.
- Experiment UUID, tick/stage, hashes and safe enum diagnostics belong on spans/logs.
- Structured events use registered names and trace/span correlation. Do not attach
  arbitrary exception text/stack, argv, environment, configuration contents, MAT
  parameters, tokens, upstream sources or complete simulator logs.
- Client-side attributes are fail-closed. Collector keeps approved resource/event
  keys and replaces log bodies. It removes service-instance UUID from metric
  resources, preventing inadvertent promotion to per-process time-series labels.
- This development Collector has **no remote exporters**. OTLP ports bind only on
  host loopback. The Collector is unprivileged, read-only, CPU/memory constrained,
  and writes only under absolute `.artifacts/telemetry/local/` with rotation.
- Remote export requires explicit opt-in + HTTPS. Production additionally needs
  authenticated OTLP, TLS verification, retention/access policies and egress rules.
  Existing report-publishing authorization is not authorization to send telemetry.
- The Collector key filter is defense-in-depth, not validation of arbitrary third-
  party span names or retained values. Review each added actor/schema before enabling
  its ingestion. Never auto-ingest all process stdout or Docker/ROS logs.

Target visual backend: local Grafana + Tempo (traces) + Prometheus (metrics) + Loki
(events), with exemplars and trace/log links. These backends are **not deployed** by
phase one. Retained OTLP JSON files prove protocol delivery, not a completed dashboard.

## Run the implemented local foundation

All commands resolve paths against absolute `AP_PNC_DIR`, regardless of cwd:

```bash
export AP_PNC_DIR=/absolute/path/to/AP-PnC
uv run --project "$AP_PNC_DIR" --extra telemetry ap-pnc telemetry up
uv run --project "$AP_PNC_DIR" --extra telemetry ap-pnc telemetry smoke
# Enable CLI tracing for an actual command (not numerical engine coverage):
AP_PNC_TELEMETRY=1 uv run --project "$AP_PNC_DIR" --extra telemetry ap-pnc telemetry status
# Use the trace_id printed by smoke:
uv run --project "$AP_PNC_DIR" --extra telemetry python \
  "$AP_PNC_DIR/infra/observability/verify_local.py" TRACE_ID
uv run --project "$AP_PNC_DIR" --extra telemetry ap-pnc telemetry down
```

Defaults: `OTEL_EXPORTER_OTLP_ENDPOINT=http://127.0.0.1:4318`, trace sample ratio 1
for these low-rate CLI operations. `AP_PNC_TELEMETRY_SAMPLE_RATIO` accepts [0,1].
High-rate numerical instrumentation needs its own tested sampling/overhead policy.
Remote HTTPS additionally requires `AP_PNC_TELEMETRY_ALLOW_REMOTE=1`; it must not be
enabled implicitly by a CLI command. No per-file path environment variables.

Pinned resources: `infra/observability/{docker-compose.yaml,collector.yaml}`;
`ap_pnc/observability.py`, `ap_pnc/commands/telemetry.py`; optional Python packages
and `uv.lock`. Output: `.artifacts/telemetry/{local,evidence}/`.

## Run the real dora integration

```bash
uv run --project "$AP_PNC_DIR" --extra telemetry ap-pnc telemetry up
bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" build
# Default remains network-none / telemetry off.
AP_PNC_TELEMETRY=0 bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" run 0 matlab-vtol lyu 6
AP_PNC_TELEMETRY=1 bash "$AP_PNC_DIR/infra/sim_infra/simple_sim/stack/stack.sh" run 0 matlab-vtol lyu 6
uv run --project "$AP_PNC_DIR" python "$AP_PNC_DIR/infra/observability/verify_dora.py" ABSOLUTE_ON_JOB --baseline ABSOLUTE_OFF_JOB
```

Telemetry-enabled graph containers join only the local Collector Compose network;
`DORA_OTLP_ENDPOINT=http://otel-collector:4317`. Private model profiles/logs are not
sent. Runtime fails preflight for unapproved remote endpoints. Explicit local
port-9 outage probes are supported for acceptance. OTLP timeout is one second;
shutdown stays outside numerical work. Standalone export/check commands do not
claim graph tracing unless a subscriber was initialized.

Sidecars `otel-*.json` live at the job root, outside the authoritative scientific
run directory. They carry real span IDs, parent IDs, experiment UUID, result and
wall duration. Actual status/tick/reference hashes are read from the committed
manifest and solver-reference CSV; solve duration histograms come from the actual
CSV measurements **after** the subprocess. These are not fabricated per-tick
C++ function spans or live real-time deadline monitoring.

Native ARM64 evidence: `.artifacts/telemetry/evidence/dora-final-*.log` and per-job
`otel-verification.json`. A full 17.48 s/874-tick on/off comparison preserved all
74 non-wall-time scientific columns, configs/reference/metrics and complete actual
NMPC reference CSV byte-for-byte; only solve_time_ms is excluded. Collector-outage
full run preserved the same data and independent replay verification. A separate
v_max=12 run failed at 4.80 s/240 committed ticks with real NMPC status=4; its
process/root spans are ERROR, raw diagnostics remain, and no replay-success marker
exists. This validates failure reporting, not high-speed tracking acceptance.
The full accepted trace has 79 observed spans including native dora transport work.

## Rollout and acceptance gates

1. **Foundation**: real native ARM64 three-signal OTLP delivery; parent/child IDs,
   log correlation, metric exemplars, privacy filters, no global SDK replacement,
   default-off/no-import behavior, collector outage without changing CLI result.
2. **Rust/dora + artifacts**: real graph chain, process/decoder boundaries, metadata
   propagation, reference/hash diagnostics and sidecars are implemented. Continue
   queue/message-age measurements, launch-parent correlation and bounded overhead
   acceptance. Receipts are verified without rerunning planner/plant. No parallel
   subscriber or new trace envelope; keep Rust-first.
3. **Numerical depth**: bridge safe C++ lifecycle hooks to OTel; instrument real
   planner/reference and acados/QP/plant/actuator paths. Bind actual solver yref,
   weights, x0 and diagnostics to tick/stage + source hashes. Do not substitute
   planned omega/thrust for NMPC's actual objective reference.
4. **Local dashboards/SLOs**: trace topology, period/solve histograms, failure drill-
   down, data integrity, queue/resource health, model/architecture comparison.
5. **ROS/SITL and production**: callback/source-age measurement, clock sync, QoS and
   safety transitions; separate Linux native ARM64/AMD64/hardware acceptance.

For each layer compare telemetry off/on: identical scientific state/reference/
command columns, solver statuses and transaction decisions (excluding wall-clock
measurements). Measure p50/p95/p99/max latency, drops, allocations, bandwidth,
CPU/RSS and disk. Stress slow/unavailable Collector, full queues, sampling, process
crash, failed solver, exporter corruption, duplicate/reordered input and shutdown.
Do not relax existing scientific acceptance to make telemetry tests pass.

Reference: OpenTelemetry Python instrumentation, W3C Trace Context, OTel Collector
transform/file exporter docs and Collector release manifest; see official
<https://opentelemetry.io/docs/languages/python/instrumentation/> and
<https://github.com/open-telemetry/opentelemetry-collector-contrib/releases/tag/v0.162.0>.
