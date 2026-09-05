# /// script
# requires-python = ">=3.10,<3.13"
# dependencies = ["numpy", "open3d"]
# ///
"""Visualize a simple_sim ASCII PCD (x y z intensity) with jet coloring.

Usage:
    uv run scripts/tools/viz_pcd.py <file.pcd> [--info]
"""

import argparse

import numpy as np
import open3d as o3d


def load_ascii_pcd(path: str) -> np.ndarray:
    """Parse an ASCII PCD with fields x y z intensity into an (N, 4) array."""
    header: dict[str, list[str]] = {}
    data_start = 0
    with open(path) as f:
        for lineno, line in enumerate(f):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0]
            if key == "DATA":
                if parts[1] != "ascii":
                    raise ValueError(f"only DATA ascii supported, got {parts[1]}")
                data_start = lineno + 1
                break
            header[key] = parts[1:]

    fields = header.get("FIELDS", ["x", "y", "z"])
    if fields[:3] != ["x", "y", "z"]:
        raise ValueError(f"unexpected FIELDS: {fields}")

    points = np.loadtxt(path, skiprows=data_start, ndmin=2)
    return points[:, : min(4, points.shape[1])]


def jet(values: np.ndarray) -> np.ndarray:
    """Map values in [0, 1] to jet colors (N, 3) in [0, 1]."""
    x = np.clip(values, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4.0 * x - 3.0), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4.0 * x - 2.0), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4.0 * x - 1.0), 0.0, 1.0)
    return np.stack([r, g, b], axis=1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", help="path to the ASCII PCD file")
    parser.add_argument("--info", action="store_true",
                        help="print statistics only, skip the viewer window")
    args = parser.parse_args()

    data = load_ascii_pcd(args.pcd)
    xyz = data[:, :3]
    intensity = data[:, 3] if data.shape[1] > 3 else None

    print(f"points: {len(xyz)}")
    print(f"x range: [{xyz[:, 0].min():.2f}, {xyz[:, 0].max():.2f}]")
    print(f"y range: [{xyz[:, 1].min():.2f}, {xyz[:, 1].max():.2f}]")
    print(f"z range: [{xyz[:, 2].min():.2f}, {xyz[:, 2].max():.2f}]")
    if intensity is not None:
        print(f"intensity range: [{intensity.min():.3f}, {intensity.max():.3f}]")

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    if intensity is not None:
        cloud.colors = o3d.utility.Vector3dVector(jet(intensity))

    if args.info:
        return
    o3d.visualization.draw_geometries(
        [cloud],
        window_name=f"simple_sim: {args.pcd}",
        width=1280,
        height=720,
    )


if __name__ == "__main__":
    main()
