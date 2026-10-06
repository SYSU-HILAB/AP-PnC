//! Dora-owned subscriber/exporters, standard W3C propagation, no second provider.
use eyre::{Result, ensure};
use opentelemetry::{Context, global, propagation::TextMapPropagator, trace::TraceContextExt};
use opentelemetry_sdk::propagation::TraceContextPropagator;
use serde::Serialize;
use std::{collections::HashMap, env, fs, path::PathBuf, time::Instant};
use tracing_opentelemetry::OpenTelemetrySpanExt;

#[derive(Clone)]
struct ExperimentId(String);

pub fn configured() -> bool {
    env::var_os("DORA_OTLP_ENDPOINT").is_some()
}

pub fn enabled() -> bool {
    configured() && tracing::dispatcher::has_been_set()
}

pub fn validate_endpoint() -> Result<()> {
    if let Ok(endpoint) = env::var("DORA_OTLP_ENDPOINT") {
        ensure!(
            matches!(
                endpoint.as_str(),
                "http://otel-collector:4317"
                    | "http://otel-collector:9"
                    | "http://127.0.0.1:4317"
                    | "http://127.0.0.1:9"
            ),
            "only the local Collector endpoint (or explicit outage probe) is supported"
        );
    }
    Ok(())
}

/// Version-pinned dora 1.0.1 metadata adapter. This is its existing reserved
/// metadata codec, NOT an alternative envelope. Split on first colon: tracestate
/// vendor values may themselves contain ':' and ';'. Never extract baggage.
pub fn decode_context(serialized: &str) -> Context {
    let mut carrier = HashMap::new();
    if serialized.len() <= 2048 {
        for line in serialized.lines() {
            if let Some((key, value)) = line.split_once(':')
                && matches!(key, "traceparent" | "tracestate")
                && value.len() <= 512
            {
                // OTel Rust 0.32 does not trim W3C list-member OWS itself.
                let value = if key == "tracestate" {
                    value
                        .split(',')
                        .map(str::trim)
                        .collect::<Vec<_>>()
                        .join(",")
                } else {
                    value.to_owned()
                };
                carrier.insert(key.to_owned(), value);
            }
        }
    }
    TraceContextPropagator::new().extract(&carrier)
}

pub fn inject_environment(command: &mut std::process::Command) {
    if enabled() {
        let mut carrier = HashMap::new();
        global::get_text_map_propagator(|p| p.inject(&mut carrier));
        if let Some(parent) = carrier.get("traceparent") {
            command.env("TRACEPARENT", parent);
        }
        // The experiment id travels as a span attribute, not as an environment
        // variable: this project defines one variable, and the simulator child
        // does not read any.
        // No arbitrary environment or baggage is serialized into telemetry.
    }
}

#[derive(Serialize)]
struct Diagnostic<'a> {
    schema: &'static str,
    operation: &'a str,
    trace_id: String,
    span_id: String,
    parent_trace_id: String,
    parent_span_id: String,
    result: &'static str,
    wall_duration_s: f64,
    experiment_id: &'a str,
}

pub fn stage<T>(
    name: &'static str,
    incoming: &str,
    experiment: &str,
    work: impl FnOnce() -> Result<T>,
) -> Result<T> {
    if !enabled() {
        return work();
    }
    let parent = if incoming.is_empty() {
        Context::current()
    } else {
        decode_context(incoming)
    };
    let parent_span = parent.span().span_context().clone();
    let experiment = if experiment.is_empty() {
        Context::current()
            .get::<ExperimentId>()
            .map(|e| e.0.clone())
            .unwrap_or_default()
    } else {
        experiment.to_owned()
    };
    let span = tracing::info_span!("dora.stage", otel.name = name, ap.pnc.operation = name,
        ap.pnc.experiment.id = experiment.as_str(), ap.pnc.result = tracing::field::Empty,
        ap.pnc.error.type = tracing::field::Empty, ap.pnc.sim_time_ns = tracing::field::Empty,
        ap.pnc.tick = tracing::field::Empty, ap.pnc.solver.status = tracing::field::Empty,
        ap.pnc.reference.sha256 = tracing::field::Empty);
    if parent_span.is_valid() {
        span.set_parent(parent)?;
    }
    let _entered = span.enter();
    // dora's send_output injects Context::current(), so attach this exact scope.
    let context = span.context().with_value(ExperimentId(experiment.clone()));
    let own = context.span().span_context().clone();
    let _attached = context.attach();
    let start = Instant::now();
    let result = work();
    let outcome = if result.is_ok() { "ok" } else { "error" };
    span.record("ap.pnc.result", outcome);
    if result.is_err() {
        span.record("ap.pnc.error.type", "StageError");
        span.set_status(opentelemetry::trace::Status::error("stage failed"));
        tracing::info!(
            monotonic_counter.ap_pnc_stage_failures = 1_u64,
            ap.pnc.operation = name
        );
    }
    let duration = start.elapsed().as_secs_f64();
    tracing::info!(
        monotonic_counter.ap_pnc_stage_calls = 1_u64,
        histogram.ap_pnc_stage_duration = duration,
        ap.pnc.operation = name,
        ap.pnc.result = outcome
    );
    let diagnostic = Diagnostic {
        schema: "ap-pnc/dora-otel/v1",
        operation: name,
        trace_id: own.trace_id().to_string(),
        span_id: own.span_id().to_string(),
        parent_trace_id: parent_span.trace_id().to_string(),
        parent_span_id: parent_span.span_id().to_string(),
        result: outcome,
        wall_duration_s: duration,
        experiment_id: &experiment,
    };
    // Out of the numerical/control path. Diagnostic IO may not change the work result.
    if write_diagnostic(name, &diagnostic).is_err() {
        tracing::warn!("telemetry sidecar write failed");
    }
    result
}

/// Post-process actual logged diagnostics; never re-run a plant or fabricate
/// kernel spans. Histograms are derived from measured solve_time_ms in steps.csv.
pub fn observe_run(dir: &std::path::Path) -> Result<()> {
    if !enabled() {
        return Ok(());
    }
    let manifest: serde_json::Value =
        serde_json::from_slice(&fs::read(dir.join("manifest.json"))?)?;
    let span = tracing::Span::current();
    if let Some(t) = manifest["time_ns"].as_i64() {
        span.record("ap.pnc.sim_time_ns", t);
    }
    if let Some(tick) = manifest["committed_steps"].as_u64() {
        span.record("ap.pnc.tick", tick);
    }
    let mut references = csv::Reader::from_path(dir.join("nmpc_reference.csv"))?;
    let field = references
        .headers()?
        .iter()
        .position(|h| h == "solver_status");
    let mut last_status = None;
    if let Some(field) = field {
        for row in references.records() {
            last_status = Some(row?[field].parse::<i64>()?);
        }
    }
    if let Some(status) = last_status {
        span.record("ap.pnc.solver.status", status);
    }
    use sha2::{Digest, Sha256};
    let digest = format!(
        "{:x}",
        Sha256::digest(fs::read(dir.join("nmpc_reference.csv"))?)
    );
    span.record("ap.pnc.reference.sha256", digest.as_str());
    let mut reader = csv::Reader::from_path(dir.join("steps.csv"))?;
    let field = reader.headers()?.iter().position(|h| h == "solve_time_ms");
    if let Some(field) = field {
        for row in reader.records() {
            let seconds = row?[field].parse::<f64>()? * 0.001;
            ensure!(
                seconds.is_finite() && seconds >= 0.0,
                "invalid logged solver duration"
            );
            tracing::info!(
                histogram.ap_pnc_nmpc_solve_duration_seconds = seconds,
                ap.pnc.operation = "nmpc.solve"
            );
        }
    }
    Ok(())
}

pub fn child<T>(name: &'static str, work: impl FnOnce() -> Result<T>) -> Result<T> {
    stage(name, "", "", work)
}

fn write_diagnostic(name: &str, diagnostic: &Diagnostic<'_>) -> Result<()> {
    let root = PathBuf::from(env::var("AP_PNC_DIR")?);
    ensure!(root.is_absolute(), "absolute project root required");
    let dir = root.join(".artifacts/simple-stack/job");
    let file = dir.join(format!("otel-{}.json", name.replace('.', "-")));
    let tmp = file.with_extension("json.tmp");
    fs::write(&tmp, serde_json::to_vec_pretty(diagnostic)?)?;
    fs::rename(tmp, file)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn existing_dora_codec_roundtrip_and_baggage_filter() {
        let context = decode_context(
            "traceparent:00-0af7651916cd43dd8448eb211c80319c-b7ad6b7169203331-01\ntracestate:vendora=t61, vendorb=a:b;c\nbaggage:private=value\n",
        );
        assert!(context.span().span_context().is_valid());
        assert_eq!(
            context.span().span_context().trace_id().to_string(),
            "0af7651916cd43dd8448eb211c80319c"
        );
        assert!(
            context
                .span()
                .span_context()
                .trace_state()
                .header()
                .contains("a:b;c")
        );
        let mut out = HashMap::new();
        TraceContextPropagator::new().inject_context(&context, &mut out);
        assert!(!out.contains_key("baggage"));
    }
    #[test]
    fn malformed_or_oversized_context_is_not_a_parent() {
        assert!(
            !decode_context("traceparent:bad")
                .span()
                .span_context()
                .is_valid()
        );
        assert!(
            !decode_context(&"x".repeat(2049))
                .span()
                .span_context()
                .is_valid()
        );
    }
}
