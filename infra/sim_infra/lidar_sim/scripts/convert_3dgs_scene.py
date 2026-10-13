#!/usr/bin/env python3
"""Convert a raw 3D Gaussian Splatting PLY into an xyz point cloud for lidar_sim.

Offline/host-side tool, not part of the ROS2 package: a 3DGS training run
emits a PLY with one row per Gaussian (position, opacity, scale, rotation,
spherical-harmonic color), stored pre-activation. This script keeps only the
Gaussians that behave like solid surface (opaque, small), recovers the ground
plane by RANSAC so the result is Z-up with ground at z=0, and writes a plain
x,y,z PLY that `lidar_sim.geometry.load_xyz_ply` / `PointCloudScene` can load.

The source scene's metric scale is whatever its own capture pipeline produced
(for Mip-NeRF360-style scenes this is close to real-world metres but not
guaranteed exact) — sanity-check the result against the flight envelope and
adjust --scale if the drone's circuit looks too tight or too loose.
"""

import argparse
import sys
from pathlib import Path

import numpy as np


def read_raw_3dgs_ply(path):
    """Read every float32 vertex property of a 3DGS training-output PLY."""
    with open(path, "rb") as handle:
        header, names, count, fmt = [], [], None, None
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("PLY header truncated")
            header.append(line)
            tokens = line.split()
            if tokens[:2] == [b"format", b"binary_little_endian"]:
                fmt = "binary_little_endian"
            elif tokens[:2] == [b"element", b"vertex"]:
                count = int(tokens[2])
            elif tokens[:1] == [b"element"] and count is not None and b"vertex" not in tokens:
                raise ValueError("expected a single vertex element; got a mesh-shaped PLY")
            elif tokens[:1] == [b"property"]:
                if tokens[1] != b"float":
                    raise ValueError(f"unsupported property type {tokens[1]!r}; expected float32")
                names.append(tokens[-1].decode())
            elif tokens[:1] == [b"end_header"]:
                break
        if fmt != "binary_little_endian" or count is None:
            raise ValueError("expected a binary_little_endian PLY with a vertex count")
        dtype = np.dtype([(name, "<f4") for name in names])
        data = np.fromfile(handle, dtype=dtype, count=count)
    if len(data) != count:
        raise ValueError("PLY vertex data truncated")
    return data


def fit_ground_plane(points, rng, iterations=300, sample_size=20000):
    """RANSAC plane through the densest near-planar cluster (the ground)."""
    sample = points[rng.choice(len(points), size=min(len(points), sample_size), replace=False)]
    diag = np.linalg.norm(points.max(axis=0) - points.min(axis=0))
    threshold = 0.01 * diag
    best_count, best_plane = -1, None
    for _ in range(iterations):
        p0, p1, p2 = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        inliers = np.abs((sample - p0) @ normal) < threshold
        if inliers.sum() > best_count:
            best_count, best_plane = inliers.sum(), (p0, normal)
    if best_plane is None:
        raise ValueError("RANSAC failed to find a ground plane; scene may be degenerate")
    p0, normal = best_plane
    inlier_mask = np.abs((points - p0) @ normal) < threshold * 1.5
    inliers = points[inlier_mask]
    centroid = inliers.mean(axis=0)
    refine_sample = inliers[rng.choice(len(inliers), size=min(len(inliers), sample_size), replace=False)]
    _, _, vt = np.linalg.svd(refine_sample - centroid, full_matrices=False)
    normal = vt[-1] / np.linalg.norm(vt[-1])
    if (points.mean(axis=0) - centroid) @ normal < 0:
        normal = -normal
    return centroid, normal


def alignment_rotation(normal, target=np.array([0.0, 0.0, 1.0])):
    """Rotation matrix mapping `normal` onto `target` (Rodrigues' formula)."""
    v = np.cross(normal, target)
    c = np.dot(normal, target)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + skew + skew @ skew / (1 + c)


def write_xyz_ply(path, points):
    points = np.ascontiguousarray(points, dtype="<f4")
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\n"
              "property float x\nproperty float y\nproperty float z\nend_header\n")
    with open(path, "wb") as handle:
        handle.write(header.encode())
        handle.write(points.tobytes())


def convert(input_path, output_path, *, opacity_threshold=0.5, max_scale=0.3,
           crop_radius_m=18.0, min_height_m=-0.3, max_height_m=10.0, scale=1.0, seed=0):
    data = read_raw_3dgs_ply(input_path)
    xyz = np.column_stack([data["x"], data["y"], data["z"]])
    opacity = 1 / (1 + np.exp(-data["opacity"]))
    gaussian_scale = np.exp(np.column_stack([data["scale_0"], data["scale_1"], data["scale_2"]])).max(axis=1)
    solid = (opacity > opacity_threshold) & (gaussian_scale < max_scale) & np.isfinite(xyz).all(axis=1)
    points = xyz[solid]
    if len(points) < 1000:
        raise ValueError("fewer than 1000 Gaussians pass the opacity/scale filter; loosen thresholds")

    rng = np.random.default_rng(seed)
    centroid, normal = fit_ground_plane(points, rng)
    rotation = alignment_rotation(normal)
    aligned = (points - centroid) @ rotation.T * scale

    radius = np.hypot(aligned[:, 0], aligned[:, 1])
    keep = (radius < crop_radius_m) & (aligned[:, 2] >= min_height_m) & (aligned[:, 2] <= max_height_m)
    result = aligned[keep]
    if len(result) < 1000:
        raise ValueError("fewer than 1000 points survive the crop; widen --crop-radius-m/--max-height-m")
    write_xyz_ply(output_path, result)
    return len(data), len(points), len(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Raw 3DGS training-output PLY (binary_little_endian)")
    parser.add_argument("output", type=Path, help="Destination xyz PLY for lidar_sim's point_cloud scene")
    parser.add_argument("--opacity-threshold", type=float, default=0.5)
    parser.add_argument("--max-scale", type=float, default=0.3, help="Drop Gaussians wider than this (metres)")
    parser.add_argument("--crop-radius-m", type=float, default=18.0)
    parser.add_argument("--min-height-m", type=float, default=-0.3)
    parser.add_argument("--max-height-m", type=float, default=10.0)
    parser.add_argument("--scale", type=float, default=1.0, help="Uniform rescale after ground alignment")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    total, solid, kept = convert(args.input, args.output, opacity_threshold=args.opacity_threshold,
                                 max_scale=args.max_scale, crop_radius_m=args.crop_radius_m,
                                 min_height_m=args.min_height_m, max_height_m=args.max_height_m,
                                 scale=args.scale, seed=args.seed)
    print(f"{total} Gaussians -> {solid} solid -> {kept} written to {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
