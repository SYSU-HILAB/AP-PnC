"""Data output helpers for the benchmark pipeline (npz core, CSV at the edge)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from tooling.env import artifact_path, project_path


def save_npz(path: str | Path, records: dict[str, np.ndarray], meta: dict | None = None) -> Path:
    """Save logged signals (and optional metadata) as a compressed npz."""
    path = artifact_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {k: np.asarray(v) for k, v in records.items()}
    if meta is not None:
        payload["_meta_json"] = np.array(json.dumps(meta))
    np.savez_compressed(path, **payload)
    return path


def load_npz(path: str | Path) -> tuple[dict[str, np.ndarray], dict]:
    """Load an npz written by :func:`save_npz`."""
    path = project_path(path)
    with np.load(path, allow_pickle=False) as npz:
        records = {k: npz[k] for k in npz.files if k != "_meta_json"}
        meta = json.loads(str(npz["_meta_json"])) if "_meta_json" in npz.files else {}
    return records, meta


def save_json(path: str | Path, obj: dict) -> Path:
    """Write a dict as pretty JSON."""
    path = artifact_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, default=float))
    return path


def to_csv(path: str | Path, records: dict[str, np.ndarray]) -> Path:
    """Edge-only CSV export (e.g. for paper_plots); not used by the core loop."""
    path = artifact_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = [k for k, v in records.items() if np.asarray(v).ndim <= 2]
    cols = []
    header = []
    for k in keys:
        arr = np.asarray(records[k], dtype=float)
        if arr.ndim == 1:
            cols.append(arr[:, None])
            header.append(k)
        else:
            cols.append(arr)
            header.extend(f"{k}_{i}" for i in range(arr.shape[1]))
    table = np.column_stack(cols) if cols else np.empty((0, 0))
    np.savetxt(path, table, delimiter=",", header=",".join(header), comments="")
    return path
