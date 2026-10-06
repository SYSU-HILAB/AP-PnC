"""Verify REAL dora graph spans in Collector files and off/on scientific identity."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

from tooling.env import artifact_path, get_git_root

OPERATIONS = (
    "experiment.simulate",
    "cpp.simulator",
    "replay.export",
    "replay.verify",
    "rerun.decoder",
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scientific_run(job):
    found = list((job / "benchmark").glob("simple_sim_*/manifest.json"))
    assert len(found) == 1, "expected one scientific run"
    return found[0].parent


def csv_rows(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def compare_science(run, baseline):
    for name in ("simple_sim.yaml", "planning.yaml", "nmpc.yaml", "reference.yaml", "metrics.json"):
        assert sha(run / name) == sha(baseline / name), f"scientific source differs: {name}"
    left, right = csv_rows(baseline / "steps.csv"), csv_rows(run / "steps.csv")
    assert len(left) == len(right) and left
    for a, b in zip(left, right, strict=True):
        for key in a:
            if key != "solve_time_ms":
                assert a[key] == b[key], f"scientific value changed: {key} at tick {a['tick']}"
    assert sha(run / "nmpc_reference.csv") == sha(baseline / "nmpc_reference.csv")
    return {
        "ticks": len(left),
        "columns": len(left[0]) - 1,
        "excluded_only": "solve_time_ms",
        "solver_reference_sha256": sha(run / "nmpc_reference.csv"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("job", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument(
        "--science-only",
        action="store_true",
        help="Outage comparison; makes no OTLP-delivery claim",
    )
    parser.add_argument("--expected-result", choices=("ok", "error"), default="ok")
    args = parser.parse_args()
    if not args.job.is_absolute() or (args.baseline and not args.baseline.is_absolute()):
        parser.error("absolute jobs required")
    job = artifact_path(args.job)
    if args.science_only:
        if not args.baseline:
            parser.error("--science-only requires --baseline")
        run = scientific_run(job)
        assert (
            json.loads((run / "manifest.json").read_text())["status"] == "finished"
            and (run / "verified.json").is_file()
        )
        comparison = compare_science(run, scientific_run(artifact_path(args.baseline)))
        report = {
            "status": "passed",
            "scope": "scientific outage identity ONLY, not successful OTLP delivery",
            "comparison": comparison,
        }
        (job / "otel-outage-comparison.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        return
    success = args.expected_result == "ok"
    wanted = OPERATIONS if success else OPERATIONS[:2]
    diagnostics = {
        n: json.loads((job / ("otel-" + n.replace(".", "-") + ".json")).read_text()) for n in wanted
    }
    ids = {d["trace_id"] for d in diagnostics.values()}
    assert len(ids) == 1 and "0" * 32 not in ids, "broken/invalid context propagation"
    trace_id = next(iter(ids))
    collector = artifact_path(get_git_root() / ".artifacts/telemetry/local")
    trace_file = collector / "traces.jsonl"
    records = [json.loads(line) for line in trace_file.read_text().splitlines() if line.strip()]
    spans = [
        s
        for r in records
        for resource in r["resourceSpans"]
        for scope in resource["scopeSpans"]
        for s in scope["spans"]
        if s["traceId"] == trace_id
    ]
    by_id = {s["spanId"]: s for s in spans}
    for name, d in diagnostics.items():
        span = by_id[d["span_id"]]
        assert span["name"] == name and d["result"] == args.expected_result
        assert int(span["endTimeUnixNano"]) >= int(span["startTimeUnixNano"]) > 0
        if d["parent_span_id"] != "0" * 16:
            assert span["parentSpanId"] == d["parent_span_id"]
        if not success:
            assert span["status"]["code"] == 2
    assert (
        diagnostics["cpp.simulator"]["parent_span_id"]
        == diagnostics["experiment.simulate"]["span_id"]
    )
    run = scientific_run(job)
    manifest = json.loads((run / "manifest.json").read_text())
    assert (manifest["status"] == "finished") == success
    assert (run / "verified.json").is_file() == success
    cpp_span = by_id[diagnostics["cpp.simulator"]["span_id"]]
    observed = {a["key"]: next(iter(a["value"].values())) for a in cpp_span["attributes"]}
    assert int(observed["ap.pnc.tick"]) == manifest["committed_steps"]
    assert int(observed["ap.pnc.sim_time_ns"]) == manifest["time_ns"]
    actual_reference = csv_rows(run / "nmpc_reference.csv")
    assert int(observed["ap.pnc.solver.status"]) == int(actual_reference[-1]["solver_status"])
    assert observed["ap.pnc.reference.sha256"] == sha(run / "nmpc_reference.csv")
    checks = {"actual_solver_diagnostic_link": True}
    if success:
        assert (
            diagnostics["replay.export"]["parent_span_id"]
            == diagnostics["experiment.simulate"]["span_id"]
        )
        assert (
            diagnostics["replay.verify"]["parent_span_id"]
            == diagnostics["replay.export"]["span_id"]
        )
        assert (
            diagnostics["rerun.decoder"]["parent_span_id"]
            == diagnostics["replay.verify"]["span_id"]
        )
        assert (
            diagnostics["experiment.simulate"]["experiment_id"]
            == diagnostics["replay.export"]["experiment_id"]
            == diagnostics["replay.verify"]["experiment_id"]
        )
        metric_file = collector / "metrics.jsonl"
        metric_messages = [
            json.loads(line) for line in metric_file.read_text().splitlines() if line.strip()
        ]
        names = {
            m["name"]
            for message in metric_messages
            for resource in message["resourceMetrics"]
            for scope in resource["scopeMetrics"]
            for m in scope["metrics"]
        }
        assert {
            "ap_pnc_stage_calls",
            "ap_pnc_stage_duration",
            "ap_pnc_nmpc_solve_duration_seconds",
        } <= names, "native dora MetricsLayer export missing"
        checks["native_metrics"] = True
    else:
        assert not (job / "otel-replay-export.json").exists(), "failed run reached exporter"
        assert (job / "simulator.log").stat().st_size > 0, "raw failure evidence lost"
    if args.baseline:
        baseline = scientific_run(artifact_path(args.baseline))
        assert not list(args.baseline.glob("otel-*.json")), "baseline was instrumented"
        checks["off_on_identical"] = compare_science(run, baseline)
    report = {
        "status": "passed",
        "trace_id": trace_id,
        "job": str(job),
        "operations": list(diagnostics),
        "observed_collector_spans": len(spans),
        "checks": checks,
        "scientific_status": manifest["status"],
        "scope": "real dora graph, C++ subprocess boundary, replay and independent decoder; NOT C++ solver/RK4 internal spans",
        "source_sha256": {
            "collector_traces": sha(trace_file),
            "manifest": sha(run / "manifest.json"),
        },
    }
    output = job / "otel-verification.json"
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
