"""Build a labelled synthetic ring from one authorized MatrixCity road quadrant.

Linux/Docker only. Axis reflections transform positions, covariance rotations,
normals AND real spherical-harmonic coefficients; the original PLY is unchanged.
"""

import json

import numpy as np
from infra.sim_infra.lidar_sim.scripts.prepare_city_scene import load_gaussians
from PIL import Image, ImageDraw

from tooling.env import artifact_path, get_git_root

CENTER_SOURCE = np.array([-3.65, 0.90, 0.0])
RADIUS_M = 88.8
HALF_EXTENT_M = 112.0
METRES_PER_UNIT = 40.0


def reflect_gaussians(data, signs):
    """Reflect about the prepared origin without changing Gaussian covariance."""
    signs = np.asarray(signs, dtype=float)
    if signs.shape != (3,) or not np.isin(signs, [-1, 1]).all():
        raise ValueError("three axis reflection signs required")
    out = data.copy()
    for axis, sign in zip("xyz", signs, strict=True):
        out[axis] *= sign
        if f"n{axis}" in out.dtype.names:
            out[f"n{axis}"] *= sign
    # R' = M R M is proper even when det(M)=-1. M commutes with diagonal
    # Gaussian scales, hence Sigma' = M Sigma M^T. Quaternion is wxyz.
    for i in range(3):
        out[f"rot_{i + 1}"] *= np.prod(signs) * signs[i]
    sx, sy, sz = signs
    # Real-SH parity in the standard 3DGS coefficient ordering, degrees 1..3.
    parity = [sy, sz, sx, sx * sy, sy * sz, 1, sx * sz, 1,
              sy, sx * sy * sz, sy, sz, sx, sz, sx]
    rest = sorted((n for n in out.dtype.names if n.startswith("f_rest_")),
                  key=lambda n: int(n.rsplit("_", 1)[1]))
    if len(rest) not in (0, 9, 24, 45):
        raise ValueError("unsupported SH layout; expected channel-major degrees 0..3")
    per_channel = len(rest) // 3
    for i, name in enumerate(rest):
        out[name] *= parity[i % per_channel]
    return out


def main():
    root = get_git_root()
    source = root / ".artifacts/dev-sim/matrixcity/assets/matrixcity-aerial.ply"
    output = artifact_path(".artifacts/dev-sim/city-ring/assets")
    output.mkdir(parents=True, exist_ok=True)
    data, header = load_gaussians(source)
    xyz = np.column_stack([data[k] for k in "xyz"])
    sizes = np.exp(np.column_stack([data[f"scale_{i}"] for i in range(3)]))
    opacity = 1 / (1 + np.exp(-np.clip(data["opacity"], -40, 40)))
    relative = xyz - CENTER_SOURCE
    keep = (np.isfinite(xyz).all(axis=1) & np.isfinite(sizes).all(axis=1)
            & (relative[:, 0] < 0) & (relative[:, 0] >= -2.8)
            & (relative[:, 1] > 0) & (relative[:, 1] <= 2.8)
            & (xyz[:, 2] >= -.08) & (xyz[:, 2] <= 4.5)
            & (sizes.max(axis=1) < .035) & (opacity > .12))
    quadrant = data[keep].copy()
    for i, axis in enumerate("xyz"):
        quadrant[axis] = relative[keep, i] * METRES_PER_UNIT
        quadrant[f"scale_{i}"] += np.log(METRES_PER_UNIT)
    solid = (opacity[keep] > .7) & (sizes[keep].max(axis=1) * METRES_PER_UNIT < .8)
    reflected = [reflect_gaussians(quadrant, signs) for signs in
                 ([1, 1, 1], [-1, 1, 1], [1, -1, 1], [-1, -1, 1])]
    with (output / "city.ply").open("wb") as target:
        for line in header:
            target.write(f"element vertex {4 * len(quadrant)}\n".encode()
                         if line.startswith(b"element vertex ") else line)
        for part in reflected:
            part.tofile(target)
    occupied = np.concatenate([np.column_stack([p[k][solid] for k in "xyz"]) for p in reflected])
    _, indices = np.unique(np.floor(occupied / .25).astype(np.int32), axis=0, return_index=True)
    occupied = occupied[indices].astype("<f4")
    with (output / "occupancy.ply").open("wb") as target:
        target.write(("ply\nformat binary_little_endian 1.0\n"
                      f"element vertex {len(occupied)}\nproperty float x\n"
                      "property float y\nproperty float z\nend_header\n").encode())
        occupied.tofile(target)
    # Highest-z-wins image from the ACTUAL transformed Gaussians, not a drawn road.
    resolution = .16
    n = int(2 * HALF_EXTENT_M / resolution)
    image = np.zeros((n, n, 3), dtype=np.uint8)
    all_data = np.concatenate(reflected)
    p = np.column_stack([all_data[k] for k in "xyz"])
    ix = np.floor((p[:, :2] + HALF_EXTENT_M) / resolution).astype(int)
    indices = np.flatnonzero((ix >= 0).all(axis=1) & (ix < n).all(axis=1))
    indices = indices[np.argsort(p[indices, 2])]
    rgb = np.clip(.5 + .28209479177387814 * np.column_stack(
        [all_data[f"f_dc_{i}"][indices] for i in range(3)]), 0, 1)
    image[ix[indices, 1], ix[indices, 0]] = (255 * rgb).astype(np.uint8)
    picture = Image.fromarray(image[::-1])
    picture.save(output / "topdown.png")
    draw = ImageDraw.Draw(picture)
    pixels = [(n / 2 + RADIUS_M / resolution * np.cos(t),
               n / 2 - RADIUS_M / resolution * np.sin(t)) for t in np.linspace(0, 2 * np.pi, 1000)]
    draw.line(pixels, fill=(255, 160, 30), width=3)
    draw.text((15, 15), "SYNTHETIC MIRRORED SCENE | ORANGE = INTENDED ROUTE, NOT FLIGHT", fill="white")
    picture.save(output / "route-preview.png")
    metadata = {
        "schema": "ap-pnc/mirrored-city-ring/v1", "synthetic": True,
        "source": str(source.relative_to(root)), "source_gaussians": len(data),
        "source_center": CENTER_SOURCE.tolist(), "source_quadrant": "northwest",
        "source_half_extent": 2.8, "source_quadrant_gaussians": len(quadrant),
        "gaussians": 4 * len(quadrant), "occupancy_points": len(occupied),
        "metres_per_source_unit": METRES_PER_UNIT, "radius_m": RADIUS_M,
        "half_extent_m": HALF_EXTENT_M, "flight_height_m": 10.0,
        "reflection": "positions + Gaussian covariance quaternion + normals + degree-2 SH parity",
        "calibration": "approximate visual scale; synthetic mirrored road, not original surveyed city",
        "caveat": "road follows an approximate circular arc; mirrored seams and reconstruction gaps remain",
    }
    (output / "scene.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
