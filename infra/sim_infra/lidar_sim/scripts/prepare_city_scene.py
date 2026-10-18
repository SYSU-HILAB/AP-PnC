"""Prepare the existing local MatrixCity asset in a shared metre-based frame.

Run inside Linux Docker. Keeps all retained Gaussian properties (not just xyz).
The metric scale is a documented visual calibration, not survey ground truth.
"""

import json

import matplotlib
import numpy as np

from tooling.env import artifact_path, get_git_root

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_gaussians(path):
    with path.open("rb") as source:
        header = []
        fields = []
        count = None
        while True:
            line = source.readline()
            if not line:
                raise ValueError("truncated PLY header")
            header.append(line)
            tokens = line.decode("ascii").split()
            if tokens[:2] == ["format", "binary_little_endian"]:
                pass
            elif tokens[:2] == ["element", "vertex"]:
                count = int(tokens[2])
            elif tokens[:1] == ["property"]:
                if tokens[1] != "float":
                    raise ValueError("expected float Gaussian properties")
                fields.append((tokens[2], "<f4"))
            elif tokens == ["end_header"]:
                break
        offset = source.tell()
    if count is None:
        raise ValueError("missing vertex count")
    return np.memmap(path, mode="r", offset=offset, dtype=fields, shape=(count,)), header


def main():
    root = get_git_root()
    source = root / ".artifacts/dev-sim/matrixcity/assets/matrixcity-aerial.ply"
    output = artifact_path(".artifacts/dev-sim/city-loop/assets")
    output.mkdir(parents=True, exist_ok=True)
    data, header = load_gaussians(source)
    scale = 40.0
    origin = np.array([-5.0, 1.0, 0.0])
    xyz = np.column_stack([data[k] for k in ("x", "y", "z")])
    sizes = np.exp(np.column_stack([data[f"scale_{i}"] for i in range(3)]))
    opacity = 1 / (1 + np.exp(-np.clip(data["opacity"], -40, 40)))
    keep = (
        np.isfinite(xyz).all(axis=1)
        & np.isfinite(sizes).all(axis=1)
        & (xyz >= [-7.6, -0.7, -0.08]).all(axis=1)
        & (xyz <= [-2.2, 3.7, 1.8]).all(axis=1)
        & (sizes.max(axis=1) < 0.035)
        & (opacity > 0.12)
    )
    clean = data[keep].copy()
    points = (xyz[keep] - origin) * scale
    for i, axis in enumerate(("x", "y", "z")):
        clean[axis] = points[:, i]
        clean[f"scale_{i}"] += np.log(scale)
    with (output / "city.ply").open("wb") as target:
        for line in header:
            target.write(f"element vertex {len(clean)}\n".encode() if line.startswith(b"element vertex ") else line)
        clean.tofile(target)
    # Map is a separate solid-surface extraction; visual opacity is not occupancy.
    solid = (opacity[keep] > 0.7) & (sizes[keep].max(axis=1) * scale < 0.8)
    occupied = points[solid]
    cells = np.floor(occupied / 0.25).astype(np.int32)
    _, indices = np.unique(cells, axis=0, return_index=True)
    occupied = occupied[indices].astype("<f4")
    with (output / "occupancy.ply").open("wb") as target:
        target.write(("ply\nformat binary_little_endian 1.0\n"
                      f"element vertex {len(occupied)}\nproperty float x\n"
                      "property float y\nproperty float z\nend_header\n").encode())
        occupied.tofile(target)
    # Survey-like top view is only for route selection, not Gaussian rendering.
    color = np.clip(0.5 + 0.28209479177387814 * np.column_stack(
        [clean[f"f_dc_{i}"] for i in range(3)]), 0, 1)
    lo, hi = np.array([-104, -68]), np.array([112, 108])
    res = 0.12
    nx, ny = np.ceil((hi - lo) / res).astype(int)
    pixel = np.floor((points[:, :2] - lo) / res).astype(int)
    valid = (pixel >= 0).all(axis=1) & (pixel < [nx, ny]).all(axis=1)
    idx = np.where(valid)[0]
    idx = idx[np.argsort(points[idx, 2])]
    image = np.full((ny, nx, 3), 0.04)
    height = np.full((ny, nx), np.nan)
    image[pixel[idx, 1], pixel[idx, 0]] = color[idx]
    height[pixel[idx, 1], pixel[idx, 0]] = points[idx, 2]
    for name, values in (("topdown", image), ("height", height)):
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.imshow(values, origin="lower", extent=[lo[0], hi[0], lo[1], hi[1]],
                  **({"cmap": "terrain", "vmin": 0, "vmax": 40} if name == "height" else {}))
        ax.set(xlabel="world x [m]", ylabel="world y [m]", title="MatrixCity / 40 metres per source unit")
        ax.set_xticks(np.arange(-100, 120, 10))
        ax.set_yticks(np.arange(-60, 110, 10))
        ax.grid(alpha=0.25)
        fig.savefig(output / f"{name}.png", dpi=120, bbox_inches="tight")
        plt.close(fig)
    metadata = {
        "source": str(source.relative_to(root)), "source_gaussians": len(data),
        "gaussians": len(clean), "occupancy_points": len(occupied),
        "metres_per_source_unit": scale, "source_origin": origin.tolist(),
        "calibration": "visual road-width/block-size calibration, not surveyed metres",
        "max_gaussian_sigma_m": 1.4, "opacity_min": 0.12,
        "transform": "world_m = (source_xyz - source_origin) * metres_per_source_unit",
    }
    (output / "scene.json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
