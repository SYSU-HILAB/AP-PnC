"""Run inside the stack container against a completed run; never re-simulate."""

import csv
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path(os.environ["AP_PNC_DIR"])
assert ROOT.is_absolute()
JOB = ROOT / ".artifacts/simple-stack/job"
BINARY = ROOT / ".artifacts/bin/ap-pnc-offline"
FILES = (
    "manifest.json",
    "metrics.json",
    "steps.csv",
    "reference.yaml",
    "simple_sim.yaml",
    "planning.yaml",
    "nmpc.yaml",
)


def main():
    runs = list((ROOT / ".artifacts/benchmark").glob("simple_sim_*"))
    assert len(runs) == 1
    original = runs[0]
    assert (original / "verified.json").is_file()
    files = list(FILES)
    if (original / "nmpc_reference.csv").is_file():
        files.append("nmpc_reference.csv")
    hashes = {f: hashlib.sha256((original / f).read_bytes()).hexdigest() for f in files}
    tests = []
    negative = JOB / "negative"
    negative.mkdir(exist_ok=True)

    def fixture(name):
        p = negative / name
        p.mkdir()  # Refuse to overwrite previous evidence.
        for f in files:
            shutil.copyfile(original / f, p / f)
        return p

    def invoke(role, path, success):
        proc = subprocess.run([str(BINARY), role, str(path)], capture_output=True, text=True)
        (path if path.is_dir() else path.parent).joinpath(f"{role}.log").write_text(
            proc.stdout + proc.stderr
        )
        assert (proc.returncode == 0) == success, (role, str(path), proc.stdout, proc.stderr)

    def reject(name, change):
        p = fixture(name)
        change(p)
        invoke("export", p, False)
        assert not (p / "replay.json").exists() and not (p / "verified.json").exists()
        tests.append(name)

    def manifest_status(p, status):
        data = json.loads((p / "manifest.json").read_text())
        data["status"] = status
        (p / "manifest.json").write_text(json.dumps(data))

    for status in ["running", "failed", "interrupted"]:
        reject(status, lambda p, status=status: manifest_status(p, status))

    def truncate(p):
        lines = (p / "steps.csv").read_text().splitlines()
        (p / "steps.csv").write_text("\n".join(lines[:-1]) + "\n")

    reject("truncated-csv", truncate)

    def mutate_csv(p, row, field, value):
        with (p / "steps.csv").open() as f:
            reader = csv.DictReader(f)
            names = reader.fieldnames
            data = list(reader)
        data[row][field] = value
        with (p / "steps.csv").open("w") as f:
            writer = csv.DictWriter(f, fieldnames=names)
            writer.writeheader()
            writer.writerows(data)

    reject("wrong-tick", lambda p: mutate_csv(p, 0, "tick", "1"))
    reject("wrong-time", lambda p: mutate_csv(p, 0, "time_ns", "123"))
    reject("nonfinite", lambda p: mutate_csv(p, 0, "p_x", "nan"))
    reject("state-discontinuity", lambda p: mutate_csv(p, 1, "p_x", "100"))
    reject("nonunit-quaternion", lambda p: mutate_csv(p, 0, "q_w", "2"))
    with (original / "steps.csv").open() as f:
        motor_channels = "rpm_1" in csv.DictReader(f).fieldnames
    if motor_channels:
        reject("rotor-discontinuity", lambda p: mutate_csv(p, 1, "rpm_1", "100"))
        reject("negative-rpm", lambda p: mutate_csv(p, 0, "next_rpm_1", "-1"))
        reject("invalid-duty", lambda p: mutate_csv(p, 0, "duty_1", "2"))

    def bad_metric(p):
        data = json.loads((p / "metrics.json").read_text())
        data["position_rmse_m"] = 100.0
        (p / "metrics.json").write_text(json.dumps(data))

    reject("wrong-metrics", bad_metric)
    if "nmpc_reference.csv" in files:

        def wrong_sample_time(p):
            file = p / "nmpc_reference.csv"
            with file.open() as f:
                reader = csv.DictReader(f)
                names = reader.fieldnames
                data = list(reader)
            data[-1]["sample_time_ns"] = "0"
            with file.open("w") as f:
                writer = csv.DictWriter(f, fieldnames=names)
                writer.writeheader()
                writer.writerows(data)

        with (original / "nmpc_reference.csv").open() as f:
            has_sample_time = "sample_time_ns" in csv.DictReader(f).fieldnames
        if has_sample_time:
            reject("wrong-clamped-sample-time", wrong_sample_time)

        def wrong_yb(p):
            file = p / "nmpc_reference.csv"
            with file.open() as f:
                reader = csv.DictReader(f)
                names = reader.fieldnames
                data = list(reader)
            manifest = json.loads((p / "manifest.json").read_text())
            yb_column = "yref_7" if manifest.get("nmpc_solver_abi") == 3 else "yref_10"
            data[0][yb_column] = str(-float(data[0][yb_column]))
            with file.open("w") as f:
                writer = csv.DictWriter(f, fieldnames=names)
                writer.writeheader()
                writer.writerows(data)

        reject("solver-yb-flip", wrong_yb)

        if json.loads((original / "manifest.json").read_text()).get("nmpc_solver_abi") == 3:

            def wrong_period(p):
                file = p / "nmpc_reference.csv"
                with file.open() as f:
                    reader = csv.DictReader(f)
                    names, data = reader.fieldnames, list(reader)
                data[0]["published_command_0"] = str(float(data[0]["published_command_0"]) + 0.1)
                with file.open("w") as f:
                    writer = csv.DictWriter(f, fieldnames=names)
                    writer.writeheader()
                    writer.writerows(data)

            reject("wrong-command-integration", wrong_period)

        def bad_horizon(p):
            file = p / "nmpc_reference.csv"
            lines = file.read_text().splitlines()
            file.write_text("\n".join(lines[:-1]) + "\n")

        reject("truncated-horizon-reference", bad_horizon)

    # Re-export from the same immutable scientific data, with the simulator
    # executable disabled: the converter cannot secretly launch another run.
    simulator = ROOT / ".artifacts/colcon/install/lib/simple_sim/simple_sim_run"
    os.chmod(simulator, 0o444)
    p = fixture("independent-export")
    invoke("export", p, True)
    invoke("check-replay", p / "replay.json", True)
    tests.append("independent-export-with-simulator-disabled")
    assert {f: hashlib.sha256((original / f).read_bytes()).hexdigest() for f in files} == hashes

    # Even a matching outer SHA does not excuse an invalid RRD encoding.
    recording = p / "replay.rrd"
    content = bytearray(recording.read_bytes())
    content[:4] = b"BAD!"
    recording.write_bytes(content)
    receipt = json.loads((p / "replay.json").read_text())
    receipt["recording_sha256"] = hashlib.sha256(content).hexdigest()
    (p / "replay.json").write_text(json.dumps(receipt))
    invoke("check-replay", p / "replay.json", False)
    assert not (p / "verified.json").exists()
    tests.append("corrupt-rrd-with-matching-sha")

    # Failed export retries invalidate both prior success markers.
    p = fixture("stale-success-marker")
    (p / "replay.json").write_text("{}")
    (p / "verified.json").write_text("{}")
    manifest_status(p, "interrupted")
    invoke("export", p, False)
    assert not (p / "replay.json").exists() and not (p / "verified.json").exists()
    tests.append("stale-success-marker")
    report = {"status": "passed", "checks": tests, "original_source_sha256": hashes}
    (JOB / "acceptance.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
