//! Offline graph: the simulator alone produces data; export never steps a plant.
mod telemetry;
use dora_node_api::{DoraNode, Event, into_vec};
use eyre::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

#[derive(Debug, Deserialize)]
struct Manifest {
    status: String,
    committed_steps: usize,
    time_ns: i64,
    control_dt_ns: i64,
    physics_dt_ns: i64,
    #[serde(default)]
    nmpc_solver_abi: Option<u32>,
}

#[derive(Debug)]
struct Run {
    dir: PathBuf,
    manifest: Manifest,
    rows: Vec<BTreeMap<String, f64>>,
    times: Vec<(i64, i64)>,
    final_reference: ([f64; 3], [f64; 3]),
    solver_references: Vec<BTreeMap<String, f64>>,
}

#[derive(Debug, Deserialize)]
struct Reference {
    order: u8,
    frame: String,
    #[serde(default)]
    terminal_policy: Option<String>,
    pieces: Vec<Piece>,
}

#[derive(Debug, Deserialize)]
struct Piece {
    duration_s: f64,
    coefficients_xyz_descending: [[f64; 6]; 3],
}

impl Reference {
    // Evaluate the simulator's persisted polynomial, never invoke a planner.
    fn sample(&self, mut t: f64) -> Result<([f64; 3], [f64; 3])> {
        ensure!(t.is_finite() && t >= 0.0, "invalid reference time");
        if self.terminal_policy.as_deref() == Some("hold_last_point") {
            t = t.min(self.pieces.iter().map(|p| p.duration_s).sum());
        }
        for (i, piece) in self.pieces.iter().enumerate() {
            if t > piece.duration_s && i + 1 < self.pieces.len() {
                t -= piece.duration_s;
                continue;
            }
            ensure!(t <= piece.duration_s + 1e-8, "reference does not cover run");
            let mut p = [0.0; 3];
            let mut v = [0.0; 3];
            for axis in 0..3 {
                for c in piece.coefficients_xyz_descending[axis] {
                    v[axis] = v[axis] * t + p[axis];
                    p[axis] = p[axis] * t + c;
                }
            }
            return Ok((p, v));
        }
        bail!("empty reference")
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct Receipt {
    schema: String,
    source: PathBuf,
    recording: PathBuf,
    steps: usize,
    final_time_ns: i64,
    source_sha256: BTreeMap<String, String>,
    recording_sha256: String,
    sdk: String,
}

fn root() -> Result<PathBuf> {
    let p = PathBuf::from(env::var("AP_PNC_DIR").wrap_err("AP_PNC_DIR required")?);
    ensure!(p.is_absolute(), "AP_PNC_DIR must be absolute");
    Ok(p.canonicalize()?)
}

fn artifact_existing(p: &Path) -> Result<PathBuf> {
    ensure!(
        p.is_absolute(),
        "absolute artifact path required: {}",
        p.display()
    );
    let p = p.canonicalize()?;
    ensure!(
        p.starts_with(root()?.join(".artifacts").canonicalize()?),
        "outside .artifacts"
    );
    Ok(p)
}

fn hash(p: &Path) -> Result<String> {
    Ok(format!("{:x}", Sha256::digest(fs::read(p)?)))
}

fn source_hashes(dir: &Path) -> Result<BTreeMap<String, String>> {
    let mut files = vec![
        "manifest.json",
        "metrics.json",
        "steps.csv",
        "reference.yaml",
        "simple_sim.yaml",
        "planning.yaml",
        "nmpc.yaml",
    ];
    if dir.join("nmpc_reference.csv").is_file() {
        files.push("nmpc_reference.csv");
    }
    files
        .into_iter()
        .map(|name| Ok((name.to_owned(), hash(&dir.join(name))?)))
        .collect()
}

fn read_run(path: &Path) -> Result<Run> {
    let dir = artifact_existing(path)?;
    let manifest: Manifest = serde_json::from_slice(&fs::read(dir.join("manifest.json"))?)?;
    ensure!(
        manifest.status == "finished",
        "refusing {} run",
        manifest.status
    );
    ensure!(manifest.committed_steps > 0, "empty run");
    ensure!(
        manifest.physics_dt_ns > 0
            && manifest.control_dt_ns > 0
            && manifest.control_dt_ns % manifest.physics_dt_ns == 0,
        "invalid periods"
    );
    let metrics: Value = serde_json::from_slice(&fs::read(dir.join("metrics.json"))?)?;
    ensure!(
        metrics["samples"].as_u64() == Some(manifest.committed_steps as u64 + 1),
        "metric sample mismatch"
    );
    let reference: Reference = serde_yaml::from_slice(&fs::read(dir.join("reference.yaml"))?)?;
    ensure!(
        reference.order == 5 && reference.frame == "world_ENU" && !reference.pieces.is_empty(),
        "unsupported reference"
    );
    ensure!(
        reference.terminal_policy.is_none()
            || reference.terminal_policy.as_deref() == Some("hold_last_point"),
        "unsupported terminal policy"
    );
    ensure!(
        reference.pieces.iter().all(|p| p.duration_s.is_finite()
            && p.duration_s > 0.0
            && p.coefficients_xyz_descending
                .iter()
                .flatten()
                .all(|c| c.is_finite())),
        "invalid reference coefficients"
    );
    let mut reader = csv::Reader::from_path(dir.join("steps.csv"))?;
    let headers = reader.headers()?.clone();
    let index = |name: &str| -> Result<usize> {
        headers
            .iter()
            .position(|v| v == name)
            .ok_or_else(|| eyre::eyre!("missing {name}"))
    };
    for prefix in ["", "next_"] {
        for field in [
            "p_x", "p_y", "p_z", "v_x", "v_y", "v_z", "q_w", "q_x", "q_y", "q_z", "w_x", "w_y",
            "w_z",
        ] {
            index(&format!("{prefix}{field}"))?;
        }
    }
    for field in [
        "ref_p_x",
        "ref_p_y",
        "ref_p_z",
        "ref_v_x",
        "ref_v_y",
        "ref_v_z",
        "specific_force_sp",
        "rate_x_sp",
        "rate_y_sp",
        "rate_z_sp",
        "imu_x",
        "imu_y",
        "imu_z",
        "aero_fx",
        "aero_fy",
        "aero_fz",
        "aero_mx",
        "aero_my",
        "aero_mz",
        "solver_status",
        "solve_time_ms",
    ] {
        index(field)?;
    }
    ensure!(
        headers
            .iter()
            .collect::<std::collections::BTreeSet<_>>()
            .len()
            == headers.len(),
        "duplicate CSV headers"
    );
    let physical_actuator = headers.iter().any(|k| k == "rpm_1");
    if physical_actuator {
        for axis in 1..=4 {
            for prefix in ["rpm_", "next_rpm_", "rpm_sp_", "duty_"] {
                index(&format!("{prefix}{axis}"))?;
            }
        }
    }
    let tick_i = index("tick")?;
    let time_i = index("time_ns")?;
    let next_i = index("next_time_ns")?;
    let mut rows: Vec<BTreeMap<String, f64>> = Vec::new();
    let mut times = Vec::new();
    for (tick, record) in reader.records().enumerate() {
        let r = record?;
        let actual_tick: usize = r[tick_i].parse()?;
        let t: i64 = r[time_i].parse()?;
        let next: i64 = r[next_i].parse()?;
        let expected = i64::try_from(tick)?
            .checked_mul(manifest.control_dt_ns)
            .ok_or_else(|| eyre::eyre!("time overflow"))?;
        ensure!(
            actual_tick == tick
                && t == expected
                && next.checked_sub(t) == Some(manifest.control_dt_ns),
            "tick/time mismatch at {tick}"
        );
        let row: BTreeMap<String, f64> = headers
            .iter()
            .zip(r.iter())
            .map(|(k, v)| Ok((k.to_owned(), v.parse::<f64>()?)))
            .collect::<Result<_>>()?;
        ensure!(
            row.values().all(|v| v.is_finite()),
            "nonfinite data at {tick}"
        );
        for prefix in ["", "next_"] {
            let n: f64 = ["w", "x", "y", "z"]
                .iter()
                .map(|axis| row[&format!("{prefix}q_{axis}")].powi(2))
                .sum();
            ensure!((n - 1.0).abs() < 1e-6, "nonunit quaternion at {tick}");
        }
        if physical_actuator {
            for axis in 1..=4 {
                ensure!(
                    row[&format!("rpm_{axis}")] >= 0.0
                        && row[&format!("next_rpm_{axis}")] >= 0.0
                        && row[&format!("rpm_sp_{axis}")] >= 0.0
                        && (-1.0..=1.0).contains(&row[&format!("duty_{axis}")]),
                    "invalid rotor state/command at {tick}"
                );
                if let Some(prev) = rows.last() {
                    ensure!(
                        (prev[&format!("next_rpm_{axis}")] - row[&format!("rpm_{axis}")]).abs()
                            < 1e-8,
                        "rotor state discontinuity at {tick}"
                    );
                }
            }
        }
        if let Some(prev) = rows.last() {
            for field in [
                "p_x", "p_y", "p_z", "v_x", "v_y", "v_z", "q_w", "q_x", "q_y", "q_z", "w_x", "w_y",
                "w_z",
            ] {
                ensure!(
                    (prev[&format!("next_{field}")] - row[field]).abs() < 1e-10,
                    "state discontinuity {field} at {tick}"
                );
            }
        }
        rows.push(row);
        times.push((t, next));
    }
    ensure!(rows.len() == manifest.committed_steps, "truncated CSV");
    ensure!(
        times.last().map(|v| v.1) == Some(manifest.time_ns),
        "final time mismatch"
    );
    // Reference and metric errors are computed from pre-step state, not next state.
    let mut pe = 0.0;
    let mut ve = 0.0;
    let mut maximum: f64 = 0.0;
    for (r, &(t, _)) in rows.iter().zip(&times) {
        let (rp, rv) = reference.sample(t as f64 * 1e-9)?;
        for (i, axis) in ["x", "y", "z"].iter().enumerate() {
            ensure!(
                (rp[i] - r[&format!("ref_p_{axis}")]).abs() < 1e-7
                    && (rv[i] - r[&format!("ref_v_{axis}")]).abs() < 1e-7,
                "persisted reference/tick mismatch"
            );
        }
        let p: f64 = ["x", "y", "z"]
            .iter()
            .map(|a| (r[&format!("p_{a}")] - r[&format!("ref_p_{a}")]).powi(2))
            .sum();
        let v: f64 = ["x", "y", "z"]
            .iter()
            .map(|a| (r[&format!("v_{a}")] - r[&format!("ref_v_{a}")]).powi(2))
            .sum();
        pe += p;
        ve += v;
        maximum = maximum.max(p.sqrt());
    }
    let final_reference = reference.sample(manifest.time_ns as f64 * 1e-9)?;
    let last = rows.last().unwrap();
    let mut final_pe = 0.0;
    for (i, axis) in ["x", "y", "z"].iter().enumerate() {
        final_pe += (last[&format!("next_p_{axis}")] - final_reference.0[i]).powi(2);
        ve += (last[&format!("next_v_{axis}")] - final_reference.1[i]).powi(2);
    }
    pe += final_pe;
    maximum = maximum.max(final_pe.sqrt());
    let status2 = rows.iter().filter(|r| r["solver_status"] == 2.0).count();
    ensure!(
        metrics["solver_status_2_count"].as_u64() == Some(status2 as u64),
        "solver count mismatch"
    );
    let n = metrics["samples"].as_u64().unwrap() as f64;
    for (key, sum) in [("position_rmse_m", pe), ("velocity_rmse_mps", ve)] {
        let actual = metrics[key]
            .as_f64()
            .ok_or_else(|| eyre::eyre!("missing {key}"))?;
        ensure!(
            actual.is_finite() && actual >= 0.0 && (actual - (sum / n).sqrt()).abs() < 1e-7,
            "inconsistent {key}"
        );
    }
    ensure!(
        metrics["position_max_m"]
            .as_f64()
            .is_some_and(|m| m.is_finite() && (m - maximum).abs() < 1e-7),
        "inconsistent max error"
    );
    let mut solver_references = Vec::new();
    if dir.join("nmpc_reference.csv").is_file() {
        let (ny, nyn, yb_start) = match manifest.nmpc_solver_abi {
            None | Some(2) => (16, 12, 9),
            Some(3) => (13, 9, 6),
            _ => bail!("unsupported NMPC reference ABI"),
        };
        let cfg: serde_yaml::Value = serde_yaml::from_slice(&fs::read(dir.join("nmpc.yaml"))?)?;
        let step_ns = (cfg["nmpc"]["traj_res_s"]
            .as_f64()
            .ok_or_else(|| eyre::eyre!("missing stage dt"))?
            * 1e9)
            .round() as i64;
        let n = (cfg["nmpc"]["horizon_s"]
            .as_f64()
            .ok_or_else(|| eyre::eyre!("missing horizon"))?
            * 1e9
            / step_ns as f64)
            .round() as usize;
        let mut reader = csv::Reader::from_path(dir.join("nmpc_reference.csv"))?;
        let headers = reader.headers()?.clone();
        ensure!(
            headers
                .iter()
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                == headers.len(),
            "duplicate solver CSV headers"
        );
        let mut required: Vec<String> = (0..ny).map(|i| format!("yref_{i}")).collect();
        if manifest.nmpc_solver_abi == Some(3) {
            required.push("solution_accepted".to_owned());
            for i in 0..4 {
                required.push(format!("u0_derivative_{i}"));
                required.push(format!("published_command_{i}"));
            }
            for i in [13, 6, 7, 8] {
                required.push(format!("initial_state_{i}"));
            }
        }
        ensure!(
            required.iter().all(|key| headers.iter().any(|h| h == key)),
            "missing solver reference/integrator fields"
        );
        for (i, record) in reader.records().enumerate() {
            let record = record?;
            let r: BTreeMap<String, f64> = headers
                .iter()
                .zip(record.iter())
                .map(|(k, v)| Ok((k.to_owned(), v.parse::<f64>()?)))
                .collect::<Result<_>>()?;
            ensure!(
                r.values().all(|v| v.is_finite()),
                "nonfinite solver reference"
            );
            let tick = i / (n + 1);
            let stage = i % (n + 1);
            let time = tick as i64 * manifest.control_dt_ns;
            ensure!(
                tick < rows.len()
                    && r["tick"] == tick as f64
                    && r["stage"] == stage as f64
                    && r["control_time_ns"] == time as f64
                    && r["reference_time_ns"] == (time + stage as i64 * step_ns) as f64
                    && r["ny"] == if stage == n { nyn as f64 } else { ny as f64 },
                "solver reference grid mismatch"
            );
            ensure!(
                r["solver_status"] == rows[tick]["solver_status"],
                "reference status mismatch"
            );
            let requested_time = r["reference_time_ns"] * 1e-9;
            if reference.terminal_policy.is_some() {
                ensure!(
                    r.contains_key("sample_time_ns"),
                    "missing clamped sample time"
                );
            }
            if let Some(&sample_ns) = r.get("sample_time_ns") {
                let duration: f64 = reference.pieces.iter().map(|p| p.duration_s).sum();
                ensure!(
                    (sample_ns - (requested_time.min(duration) * 1e9).round()).abs() <= 1.0,
                    "clamped sample time mismatch"
                );
            }
            let (p, v) = reference.sample(requested_time)?;
            for axis in 0..3 {
                ensure!(
                    (r[&format!("yref_{axis}")] - p[axis]).abs() < 1e-7
                        && (r[&format!("yref_{}", axis + 3)] - v[axis]).abs() < 1e-7,
                    "solver polynomial reference mismatch"
                );
                let name = format!("planner_yb_{}", ["x", "y", "z"][axis]);
                ensure!(
                    (r[&format!("yref_{}", axis + yb_start)] - r[&name]).abs() < 1e-12,
                    "planner to NMPC yb changed"
                );
            }
            let norm: f64 = (yb_start..yb_start + 3)
                .map(|a| r[&format!("yref_{a}")].powi(2))
                .sum();
            ensure!((norm - 1.0).abs() < 1e-8, "nonunit yb reference");
            if manifest.nmpc_solver_abi == Some(3) {
                ensure!(
                    (9..13).all(|a| r[&format!("yref_{a}")] == 0.0),
                    "command derivative reference is not zero"
                );
                ensure!(
                    r["solution_accepted"] == 1.0,
                    "unaccepted solution in committed run"
                );
                for (a, state_index) in [13, 6, 7, 8].iter().enumerate() {
                    let initial = r[&format!("initial_state_{state_index}")];
                    let derivative = r[&format!("u0_derivative_{a}")];
                    let published = r[&format!("published_command_{a}")];
                    let expected = initial + manifest.control_dt_ns as f64 * 1e-9 * derivative;
                    ensure!(
                        (published - expected).abs() < 1e-10,
                        "command was not integrated at actual control period"
                    );
                    let column = ["specific_force_sp", "rate_x_sp", "rate_y_sp", "rate_z_sp"][a];
                    ensure!(
                        (published - rows[tick][column]).abs() < 1e-10,
                        "published command differs from scientific step"
                    );
                    if tick > 0 {
                        ensure!(
                            (initial - rows[tick - 1][column]).abs() < 1e-10,
                            "command integrator memory discontinuity"
                        );
                    }
                }
            }
            solver_references.push(r);
        }
        ensure!(
            solver_references.len() == rows.len() * (n + 1),
            "truncated solver references"
        );
    }
    Ok(Run {
        dir,
        manifest,
        rows,
        times,
        final_reference,
        solver_references,
    })
}

fn vec3(r: &BTreeMap<String, f64>, prefix: &str) -> [f32; 3] {
    [
        r[&format!("{prefix}x")] as f32,
        r[&format!("{prefix}y")] as f32,
        r[&format!("{prefix}z")] as f32,
    ]
}

fn log_state(
    rec: &rerun::RecordingStream,
    r: &BTreeMap<String, f64>,
    prefix: &str,
    time_ns: i64,
) -> Result<()> {
    rec.set_time("sim_time", rerun::TimeCell::from_duration_nanos(time_ns));
    let p = vec3(r, &format!("{prefix}p_"));
    let q = rerun::Quaternion::from_xyzw([
        r[&format!("{prefix}q_x")] as f32,
        r[&format!("{prefix}q_y")] as f32,
        r[&format!("{prefix}q_z")] as f32,
        r[&format!("{prefix}q_w")] as f32,
    ]);
    rec.log(
        "world/vehicle",
        &rerun::Transform3D::from_translation_rotation(p, q),
    )?;
    rec.log(
        "world/position",
        &rerun::Points3D::new([p]).with_radii([0.06]),
    )?;
    rec.log(
        "world/velocity",
        &rerun::Arrows3D::from_vectors([vec3(r, &format!("{prefix}v_"))]).with_origins([p]),
    )?;
    for axis in ["x", "y", "z"] {
        for field in ["p", "v", "w"] {
            rec.log(
                format!("state/{field}/{axis}"),
                &rerun::Scalars::single(r[&format!("{prefix}{field}_{axis}")]),
            )?;
        }
    }
    Ok(())
}

fn export(dir: &Path) -> Result<PathBuf> {
    let dir = artifact_existing(dir)?;
    // Invalidate old success markers even if source validation fails on retry.
    for name in ["replay.json", "verified.json"] {
        let marker = dir.join(name);
        if marker.exists() {
            fs::remove_file(marker)?;
        }
    }
    let before = source_hashes(&dir)?;
    let run = read_run(&dir)?;
    let final_path = run.dir.join("replay.rrd");
    let tmp = run.dir.join("replay.rrd.partial");
    let receipt_path = run.dir.join("replay.json");
    let rec = rerun::RecordingStreamBuilder::new("ap-pnc/simple_sim/offline").save(&tmp)?;
    rec.log_static("world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;
    rec.log_static(
        "world/vehicle/body_axes",
        &rerun::Arrows3D::from_vectors([[0.4, 0.0, 0.0], [0.0, 0.4, 0.0], [0.0, 0.0, 0.4]])
            .with_origins([[0.0, 0.0, 0.0]; 3])
            .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]]),
    )?;
    rec.log_static(
        "metadata/manifest",
        &rerun::TextDocument::new(fs::read_to_string(run.dir.join("manifest.json"))?),
    )?;
    rec.log_static(
        "metadata/metrics",
        &rerun::TextDocument::new(fs::read_to_string(run.dir.join("metrics.json"))?),
    )?;
    let mut actual_path: Vec<[f32; 3]> = run.rows.iter().map(|r| vec3(r, "p_")).collect();
    actual_path.push(vec3(run.rows.last().unwrap(), "next_p_"));
    let mut reference_path: Vec<[f32; 3]> = run.rows.iter().map(|r| vec3(r, "ref_p_")).collect();
    reference_path.push(run.final_reference.0.map(|v| v as f32));
    rec.log_static(
        "world/path",
        &rerun::LineStrips3D::new([actual_path]).with_colors([[40, 160, 255]]),
    )?;
    rec.log_static(
        "world/reference_path",
        &rerun::LineStrips3D::new([reference_path]).with_colors([[255, 180, 40]]),
    )?;
    for (r, &(t, _)) in run.rows.iter().zip(&run.times) {
        log_state(&rec, r, "", t)?;
        rec.log(
            "world/reference",
            &rerun::Points3D::new([vec3(r, "ref_p_")]).with_colors([[255, 180, 40]]),
        )?;
        let error: f64 = ["x", "y", "z"]
            .iter()
            .map(|a| (r[&format!("p_{a}")] - r[&format!("ref_p_{a}")]).powi(2))
            .sum::<f64>()
            .sqrt();
        rec.log("tracking/position_error_m", &rerun::Scalars::single(error))?;
        // Optional physical-actuator channels: exported only from logged data.
        for (key, value) in r {
            if key.starts_with("rpm_")
                || key.starts_with("next_rpm_")
                || key.starts_with("duty_")
                || key.starts_with("applied_")
            {
                // next rotor state belongs at next_time_ns, never t_k.
                if key.starts_with("next_rpm_") {
                    continue;
                }
                rec.log(format!("actuator/{key}"), &rerun::Scalars::single(*value))?;
            }
        }
        for key in [
            "specific_force_sp",
            "rate_x_sp",
            "rate_y_sp",
            "rate_z_sp",
            "imu_x",
            "imu_y",
            "imu_z",
            "aero_fx",
            "aero_fy",
            "aero_fz",
            "aero_mx",
            "aero_my",
            "aero_mz",
            "solver_status",
            "solve_time_ms",
        ] {
            rec.log(format!("signals/{key}"), &rerun::Scalars::single(r[key]))?;
        }
    }
    log_state(
        &rec,
        run.rows.last().unwrap(),
        "next_",
        run.manifest.time_ns,
    )?;
    rec.log(
        "world/reference",
        &rerun::Points3D::new([run.final_reference.0.map(|v| v as f32)])
            .with_colors([[255, 180, 40]]),
    )?;
    let last = run.rows.last().unwrap();
    let final_error = ["x", "y", "z"]
        .iter()
        .enumerate()
        .map(|(i, a)| (last[&format!("next_p_{a}")] - run.final_reference.0[i]).powi(2))
        .sum::<f64>()
        .sqrt();
    rec.log(
        "tracking/position_error_m",
        &rerun::Scalars::single(final_error),
    )?;
    for axis in 1..=4 {
        let key = format!("next_rpm_{axis}");
        if let Some(rpm) = last.get(&key) {
            rec.log(
                format!("actuator/rpm_{axis}"),
                &rerun::Scalars::single(*rpm),
            )?;
        }
    }
    for row in &run.solver_references {
        if row["stage"] != 0.0 {
            continue;
        }
        rec.set_time(
            "sim_time",
            rerun::TimeCell::from_duration_nanos(row["control_time_ns"] as i64),
        );
        for (key, value) in row {
            if key.starts_with("yref_")
                || key.starts_with("planner_")
                || key.starts_with("parameter_")
            {
                rec.log(
                    format!("nmpc/reference/{key}"),
                    &rerun::Scalars::single(*value),
                )?;
            }
        }
    }
    rec.flush_blocking()?;
    drop(rec);
    ensure!(
        source_hashes(&run.dir)? == before,
        "source changed during export"
    );
    ensure!(fs::metadata(&tmp)?.len() > 0, "empty recording");
    fs::rename(tmp, &final_path)?;
    let receipt = Receipt {
        schema: "ap-pnc/offline-replay/v1".to_owned(),
        source: run.dir,
        recording: final_path.clone(),
        steps: run.manifest.committed_steps,
        final_time_ns: run.manifest.time_ns,
        source_sha256: before,
        recording_sha256: hash(&final_path)?,
        sdk: "rerun-0.38.1".to_owned(),
    };
    let temp_receipt = receipt_path.with_extension("json.tmp");
    fs::write(&temp_receipt, serde_json::to_vec_pretty(&receipt)?)?;
    fs::rename(temp_receipt, &receipt_path)?;
    Ok(receipt_path)
}

fn verify(receipt_path: &Path) -> Result<()> {
    let p = artifact_existing(receipt_path)?;
    let marker = p.parent().unwrap().join("verified.json");
    if marker.exists() {
        fs::remove_file(marker)?;
    }
    let receipt: Receipt = serde_json::from_slice(&fs::read(p)?)?;
    ensure!(
        receipt.schema == "ap-pnc/offline-replay/v1" && receipt.sdk == "rerun-0.38.1",
        "unknown replay schema/SDK"
    );
    let run = read_run(&receipt.source)?;
    ensure!(
        receipt.steps == run.manifest.committed_steps
            && receipt.final_time_ns == run.manifest.time_ns,
        "receipt tick mismatch"
    );
    ensure!(
        source_hashes(&run.dir)? == receipt.source_sha256,
        "source hash mismatch"
    );
    let recording = artifact_existing(&receipt.recording)?;
    ensure!(
        recording == run.dir.join("replay.rrd"),
        "unexpected recording location"
    );
    ensure!(
        hash(&recording)? == receipt.recording_sha256,
        "recording hash mismatch"
    );
    telemetry::child("rerun.decoder", || {
        let out = Command::new("rerun")
            .args(["rrd", "verify"])
            .arg(&recording)
            .output()?;
        fs::write(
            run.dir.join("rrd-verify.log"),
            [&out.stdout[..], &out.stderr[..]].concat(),
        )?;
        ensure!(
            out.status.success(),
            "Rerun decoder rejected recording (raw details retained in local rrd-verify.log)"
        );
        Ok(())
    })?;
    fs::write(
        run.dir.join("verified.json"),
        serde_json::to_vec_pretty(&receipt)?,
    )?;
    println!(
        "Verified {} steps, {} ns: {}",
        receipt.steps,
        receipt.final_time_ns,
        recording.display()
    );
    Ok(())
}

fn simulate(config: &Path) -> Result<PathBuf> {
    ensure!(config.is_absolute(), "absolute config required");
    let bench = root()?.join(".artifacts/benchmark");
    fs::create_dir_all(&bench)?;
    ensure!(
        fs::read_dir(&bench)?.next().is_none(),
        "simulation requires an empty per-job benchmark mount"
    );
    telemetry::child("cpp.simulator", || {
        let mut command =
            Command::new(root()?.join(".artifacts/colcon/install/lib/simple_sim/simple_sim_run"));
        command.arg("--config").arg(config);
        telemetry::inject_environment(&mut command);
        let out = command.output()?;
        // Retain raw diagnostics locally on success AND failure.
        fs::write(
            root()?.join(".artifacts/simple-stack/job/simulator.log"),
            [&out.stdout[..], &out.stderr[..]].concat(),
        )?;
        if telemetry::enabled() {
            let diagnostic = (|| -> Result<()> {
                let dirs = fs::read_dir(&bench)?.collect::<std::io::Result<Vec<_>>>()?;
                if let Some(entry) = dirs.first() {
                    telemetry::observe_run(&entry.path())?;
                }
                Ok(())
            })();
            if diagnostic.is_err() {
                tracing::warn!("logged solver telemetry unavailable");
            }
        }
        ensure!(
            out.status.success(),
            "simulator failed (raw details retained in local simulator.log)"
        );
        Ok(())
    })?;
    let dirs: Vec<PathBuf> = fs::read_dir(&bench)?
        .map(|r| r.map(|e| e.path()))
        .collect::<std::io::Result<_>>()?;
    ensure!(dirs.len() == 1, "expected exactly one simulator output");
    read_run(&dirs[0])?;
    Ok(dirs[0].clone())
}

fn send(node: &mut DoraNode, id: &str, path: &Path) -> Result<()> {
    let bytes = serde_json::to_vec(path)?;
    node.send_output(id.to_owned().into(), Default::default(), bytes)?;
    Ok(())
}

fn main() -> Result<()> {
    let mut args = env::args().skip(1);
    let role = args
        .next()
        .ok_or_else(|| eyre::eyre!("simulate|record|verify|export|check-run PATH"))?;
    if role == "export" || role == "check-run" || role == "check-replay" {
        let path = PathBuf::from(args.next().ok_or_else(|| eyre::eyre!("path required"))?);
        match role.as_str() {
            "export" => {
                println!("{}", export(&path)?.display());
            }
            "check-run" => {
                let r = read_run(&path)?;
                println!("{} valid steps", r.rows.len());
            }
            _ => verify(&path)?,
        }
        return Ok(());
    }
    telemetry::validate_endpoint()?;
    let (mut node, mut events) = DoraNode::init_from_env()?;
    // Public dora lifecycle owns subscriber and both OTLP exporters.
    let runtime = if telemetry::configured() {
        Some(
            tokio::runtime::Builder::new_multi_thread()
                .worker_threads(2)
                .enable_all()
                .build()?,
        )
    } else {
        None
    };
    let otel_guard = if let Some(rt) = &runtime {
        let _entered = rt.enter();
        opentelemetry::global::set_text_map_propagator(
            opentelemetry_sdk::propagation::TraceContextPropagator::new(),
        );
        let guard = dora_node_api::init_tracing(node.id(), node.dataflow_id())?;
        // init_tracing starts asynchronously. Do not create/export a scope before readiness.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while !(tracing::dispatcher::has_been_set()
            && guard.lock().map(|g| g.is_some()).unwrap_or(false))
        {
            ensure!(
                std::time::Instant::now() < deadline,
                "dora tracing initialization timed out"
            );
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        Some(guard)
    } else {
        None
    };
    let result = run_node(&role, &mut args, &mut node, &mut events);
    drop(events);
    drop(node);
    // Flush only after node work; never in control/RK4. Keep Tokio alive until shutdown.
    if let Some(guard) = otel_guard
        && let Ok(mut guard) = guard.lock()
    {
        drop(guard.take());
    }
    drop(runtime);
    result
}

fn run_node(
    role: &str,
    args: &mut impl Iterator<Item = String>,
    node: &mut DoraNode,
    events: &mut dora_node_api::EventStream,
) -> Result<()> {
    let experiment = node.dataflow_id().to_string();
    if role == "simulate" {
        let config = PathBuf::from(args.next().ok_or_else(|| eyre::eyre!("config required"))?);
        let incoming = env::var("TRACEPARENT")
            .map(|p| format!("traceparent:{p}\n"))
            .unwrap_or_default();
        return telemetry::stage("experiment.simulate", &incoming, &experiment, || {
            let path = simulate(&config)?;
            send(node, "run", &path)?;
            Ok(())
        });
    }
    ensure!(role == "record" || role == "verify", "unknown role {role}");
    let expected_id = if role == "record" { "run" } else { "recording" };
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, metadata } if id.as_str() == expected_id => {
                let incoming = metadata.open_telemetry_context();
                return telemetry::stage(
                    if role == "record" {
                        "replay.export"
                    } else {
                        "replay.verify"
                    },
                    &incoming,
                    &experiment,
                    || {
                        let bytes = into_vec::<u8>(&data)?;
                        let path: PathBuf = serde_json::from_slice(&bytes)?;
                        if role == "record" {
                            let receipt = export(&path)?;
                            send(node, "recording", &receipt)?;
                        } else {
                            verify(&path)?;
                        }
                        Ok(())
                    },
                );
            }
            Event::Stop(_) => bail!("stopped before {expected_id}"),
            _ => {}
        }
    }
    bail!("input closed without successful {expected_id}")
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn relative_paths_are_rejected_before_io() {
        assert!(artifact_existing(Path::new("relative/run")).is_err());
    }
    #[test]
    fn terminal_hold_is_explicit_and_repeats_endpoint() {
        let mut reference = Reference {
            order: 5,
            frame: "world_ENU".to_owned(),
            terminal_policy: None,
            pieces: vec![Piece {
                duration_s: 1.0,
                coefficients_xyz_descending: [[0.0, 0.0, 0.0, 0.0, 2.0, 3.0], [0.0; 6], [0.0; 6]],
            }],
        };
        assert!(reference.sample(1.1).is_err()); // Legacy data still rejects extrapolation.
        reference.terminal_policy = Some("hold_last_point".to_owned());
        let endpoint = reference.sample(1.0).unwrap();
        assert_eq!(endpoint, ([5.0, 0.0, 0.0], [2.0, 0.0, 0.0]));
        assert_eq!(reference.sample(1.1).unwrap(), endpoint);
        assert_eq!(reference.sample(100.0).unwrap(), endpoint);
        assert!(reference.sample(-1.0).is_err());
        assert!(reference.sample(f64::NAN).is_err());
    }
    #[test]
    fn vector_axis_order() {
        let row = BTreeMap::from([
            ("p_x".to_owned(), 1.0),
            ("p_y".to_owned(), 2.0),
            ("p_z".to_owned(), 3.0),
        ]);
        assert_eq!(vec3(&row, "p_"), [1.0, 2.0, 3.0]);
    }
}
