"""Verify actual Collector OTLP files for a synthetic probe (not solver acceptance)."""

import argparse
import hashlib
import json
import re

from tooling.env import artifact_path, get_git_root


def messages(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def attributes(items):
    return {a["key"]: next(iter(a["value"].values())) for a in items}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace_id")
    args = parser.parse_args()
    if not re.fullmatch("[a-f0-9]{32}", args.trace_id):
        parser.error("32-digit hex trace id required")
    directory = artifact_path(get_git_root() / ".artifacts/telemetry/local")
    sources = {name: directory / f"{name}.jsonl" for name in ("traces", "metrics", "logs")}
    decoded = {k: messages(v) for k, v in sources.items()}
    spans = [
        s
        for m in decoded["traces"]
        for r in m["resourceSpans"]
        for sc in r["scopeSpans"]
        for s in sc["spans"]
        if s["traceId"] == args.trace_id
    ]
    by_name = {s["name"]: s for s in spans}
    assert {"config.load", "nmpc.solve", "telemetry.smoke"} <= set(by_name)
    parent = by_name["telemetry.smoke"]["spanId"]
    assert all(by_name[n]["parentSpanId"] == parent for n in ("config.load", "nmpc.solve"))
    assert by_name["nmpc.solve"]["status"]["code"] == 2
    assert attributes(by_name["nmpc.solve"]["attributes"])["ap.pnc.solver.status"] == "4"
    assert attributes(by_name["telemetry.smoke"]["attributes"])["ap.pnc.sim_time_ns"] == "0"
    assert int(by_name["telemetry.smoke"]["startTimeUnixNano"]) > 0  # Not simulated time.
    logs = [
        record
        for m in decoded["logs"]
        for r in m["resourceLogs"]
        for sc in r["scopeLogs"]
        for record in sc["logRecords"]
        if record.get("traceId") == args.trace_id
    ]
    assert len(logs) >= 7 and all(record["body"] == {"stringValue": "ap.pnc.event"} for record in logs)
    points = []
    for message in decoded["metrics"]:
        for resource in message["resourceMetrics"]:
            assert "service.instance.id" not in attributes(resource["resource"]["attributes"])
            for scope in resource["scopeMetrics"]:
                for metric in scope["metrics"]:
                    value = metric.get("sum", metric.get("histogram", {}))
                    points.extend(value.get("dataPoints", []))
    assert points and all(
        set(attributes(p.get("attributes", []))) <= {"ap.pnc.operation", "ap.pnc.result"}
        for p in points
    )
    exemplars = [
        e for p in points for e in p.get("exemplars", []) if e.get("traceId") == args.trace_id
    ]
    assert exemplars
    assert all("probe-only private error body" not in p.read_text() for p in sources.values())
    report = {
        "status": "passed",
        "scope": "synthetic Python -> native Linux ARM64 Collector; not numeric/flight instrumentation",
        "trace_id": args.trace_id,
        "spans": len(spans),
        "correlated_logs": len(logs),
        "metric_points": len(points),
        "matching_exemplars": len(exemplars),
        "source_sha256": {
            k: hashlib.sha256(v.read_bytes()).hexdigest() for k, v in sources.items()
        },
    }
    output = artifact_path(get_git_root() / ".artifacts/telemetry/evidence/verification.json")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
