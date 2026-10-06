"""Full headless pipeline replay check, excluding diagnostic wall-time fields."""

import csv
import json
import os
import re
import subprocess
import sys
from pathlib import Path


def main() -> None:
    executable = Path(sys.argv[1]).resolve()
    root = Path(sys.argv[2]).resolve()
    runs = []
    for index in range(2):
        result = subprocess.run(
            [str(executable), "--controller", sys.argv[3]],
            env={**os.environ, "AP_PNC_DIR": str(root)},
            capture_output=True,
            text=True,
            timeout=60,
        )
        (executable.parent / f"repeatability-{index}.log").write_text(result.stdout + result.stderr)
        assert result.returncode == 0, result.stdout + result.stderr
        match = re.search(r'Output: "([^"]+)"', result.stdout)
        assert match, "missing output directory"
        path = Path(match.group(1))
        manifest = json.loads((path / "manifest.json").read_text())
        assert manifest["status"] == "finished", manifest
        with (path / "steps.csv").open() as stream:
            records = list(csv.DictReader(stream))
        for row in records:
            row.pop("solve_time_ms")
        assert len(records) == manifest["committed_steps"]
        for tick, row in enumerate(records):
            assert int(row["tick"]) == tick
            assert int(row["time_ns"]) == tick * manifest["control_dt_ns"]
            assert int(row["next_time_ns"]) == (tick + 1) * manifest["control_dt_ns"]
        runs.append((path, records))
    assert runs[0][1] == runs[1][1], "state/reference/control replay differs"
    assert (runs[0][0] / "reference.yaml").read_bytes() == (
        runs[1][0] / "reference.yaml"
    ).read_bytes()
    print(
        f"PASS full {sys.argv[3]} pipeline: {len(runs[0][1])} steps, exact replay except wall time"
    )


if __name__ == "__main__":
    main()
