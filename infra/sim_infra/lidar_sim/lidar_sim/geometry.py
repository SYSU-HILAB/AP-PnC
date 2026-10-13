"""Deterministic point-cloud map and analytic first-return cylinder ray casting."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation


def _validate_rays(origin, directions, min_range, max_range):
    """Shared ray-batch checks for every scene's `ranges` implementation."""
    origin = np.asarray(origin, dtype=float)
    directions = np.asarray(directions, dtype=float)
    if origin.shape != (3,) or directions.ndim != 2 or directions.shape[1] != 3:
        raise ValueError("origin/directions must be (3,) and (N,3)")
    if not np.isfinite(origin).all() or not np.isfinite(directions).all():
        raise ValueError("nonfinite rays")
    if not np.allclose(np.linalg.norm(directions, axis=1), 1, atol=1e-8):
        raise ValueError("ray directions must have unit norm")
    if not np.isfinite([min_range, max_range]).all() or not 0 < min_range < max_range:
        raise ValueError("invalid range bounds")
    return origin, directions


@dataclass(frozen=True)
class Pillar:
    x: float
    y: float
    radius: float
    height: float
    base_z: float = 0.0

    def __post_init__(self):
        if not np.isfinite([self.x, self.y, self.radius, self.height, self.base_z]).all():
            raise ValueError("pillar values must be finite")
        if self.radius <= 0 or self.height <= 0:
            raise ValueError("pillar radius and height must be positive")


@dataclass(frozen=True)
class Scene:
    pillars: tuple[Pillar, ...]
    ground_z: float = 0.0
    half_extent: float = 30.0
    map_resolution: float = 0.25

    def __post_init__(self):
        if not np.isfinite([self.ground_z, self.half_extent, self.map_resolution]).all():
            raise ValueError("scene values must be finite")
        if self.half_extent <= 0 or self.map_resolution <= 0:
            raise ValueError("scene extent and resolution must be positive")
        if (2 * self.half_extent / self.map_resolution + 1) ** 2 > 500000:
            raise ValueError("map resolution produces too many ground points")
        if len(self.pillars) > 100:
            raise ValueError("too many pillars")

    @classmethod
    def from_dict(cls, data):
        return cls(
            tuple(Pillar(**item) for item in data["pillars"]),
            ground_z=float(data.get("ground_z", 0.0)),
            half_extent=float(data.get("half_extent", 30.0)),
            map_resolution=float(data.get("map_resolution", 0.25)),
        )

    def point_cloud(self):
        """Surface samples for map display; scans do not cheat using this cloud."""
        step = self.map_resolution
        grid = np.arange(-self.half_extent, self.half_extent + step / 2, step)
        x, y = np.meshgrid(grid, grid)
        parts = [np.column_stack((x.ravel(), y.ravel(), np.full(x.size, self.ground_z)))]
        for pillar in self.pillars:
            count = max(12, int(np.ceil(2 * np.pi * pillar.radius / step)))
            levels = max(2, int(np.ceil(pillar.height / step)) + 1)
            if count * levels > 100000 or (2 * pillar.radius / step + 1) ** 2 > 100000:
                raise ValueError("pillar surface produces too many points")
            angles, height = np.meshgrid(
                np.linspace(-np.pi, np.pi, count, endpoint=False),
                np.linspace(pillar.base_z, pillar.base_z + pillar.height, levels),
            )
            parts.append(np.column_stack((
                pillar.x + pillar.radius * np.cos(angles.ravel()),
                pillar.y + pillar.radius * np.sin(angles.ravel()), height.ravel(),
            )))
            # Caps use a Cartesian grid cropped to the disc.
            q = np.arange(-pillar.radius, pillar.radius + step / 2, step)
            cx, cy = np.meshgrid(q, q)
            inside = cx * cx + cy * cy <= pillar.radius ** 2
            for z in (pillar.base_z, pillar.base_z + pillar.height):
                parts.append(np.column_stack((cx[inside] + pillar.x, cy[inside] + pillar.y,
                                              np.full(np.sum(inside), z))))
        return np.concatenate(parts).astype(np.float32)

    def ranges(self, origin, directions, min_range, max_range):
        """Intersect finite cylinders (including caps) and bounded ground.

        Directions and distances are in world metres. The nearest geometric hit
        wins, including a near hit inside the minimum range: never see through it.
        """
        origin, directions = _validate_rays(origin, directions, min_range, max_range)
        distances = np.full(len(directions), np.inf)
        dx, dy, dz = directions.T

        def plane_hit(z, pillar=None):
            t = np.divide(z - origin[2], dz, out=np.full(len(dz), np.inf), where=np.abs(dz) > 1e-12)
            finite = np.isfinite(t)
            tx = origin[0] + np.where(finite, t, 0) * dx
            ty = origin[1] + np.where(finite, t, 0) * dy
            if pillar is None:
                inside = (np.abs(tx) <= self.half_extent) & (np.abs(ty) <= self.half_extent)
            else:
                inside = (tx - pillar.x) ** 2 + (ty - pillar.y) ** 2 <= pillar.radius ** 2 + 1e-10
            np.minimum(distances, np.where(finite & (t > 1e-9) & inside, t, np.inf), out=distances)

        plane_hit(self.ground_z)
        for pillar in self.pillars:
            ox, oy = origin[0] - pillar.x, origin[1] - pillar.y
            a = dx * dx + dy * dy
            b = 2 * (ox * dx + oy * dy)
            c = ox * ox + oy * oy - pillar.radius ** 2
            discriminant = b * b - 4 * a * c
            eligible = (a > 1e-12) & (discriminant >= 0)
            root = np.sqrt(np.maximum(discriminant, 0))
            for sign in (-1, 1):
                t = np.divide(-b + sign * root, 2 * a, out=np.full(len(a), np.inf), where=eligible)
                z = origin[2] + np.where(np.isfinite(t), t, 0) * dz
                valid = eligible & (t > 1e-9) & (z >= pillar.base_z) & (z <= pillar.base_z + pillar.height)
                np.minimum(distances, np.where(valid, t, np.inf), out=distances)
            plane_hit(pillar.base_z, pillar)
            plane_hit(pillar.base_z + pillar.height, pillar)
        distances[(distances < min_range) | (distances > max_range)] = np.inf
        return distances


def load_xyz_ply(path):
    """Read vertex x,y,z from an ASCII or binary_little_endian PLY; other properties ignored."""
    with open(path, "rb") as handle:
        line = handle.readline()
        if line.strip() != b"ply":
            raise ValueError("not a PLY file")
        fmt, count, names = None, None, []
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("PLY header truncated")
            tokens = line.split()
            if tokens[:2] == [b"format", b"ascii"]:
                fmt = "ascii"
            elif tokens[:2] == [b"format", b"binary_little_endian"]:
                fmt = "binary_little_endian"
            elif tokens[:2] == [b"element", b"vertex"]:
                count = int(tokens[2])
            elif tokens[:1] == [b"element"]:
                raise ValueError("PLY must contain only a vertex element (no faces/edges)")
            elif tokens[:1] == [b"property"]:
                names.append(tokens[-1].decode())
            elif tokens[:1] == [b"end_header"]:
                break
        if fmt is None or count is None:
            raise ValueError("PLY header missing format/vertex count")
        if "x" not in names or "y" not in names or "z" not in names:
            raise ValueError("PLY vertices must have x, y, z properties")
        if fmt == "ascii":
            rows = [handle.readline().split() for _ in range(count)]
            data = {name: np.array([float(row[i]) for row in rows]) for i, name in enumerate(names)}
        else:
            dtype = np.dtype([(name, "<f4") for name in names])
            data = np.fromfile(handle, dtype=dtype, count=count)
            data = {name: data[name] for name in names}
        points = np.column_stack((data["x"], data["y"], data["z"])).astype(np.float32)
    if not np.isfinite(points).all():
        raise ValueError("PLY contains nonfinite vertices")
    return points


class PointCloudScene:
    """Ray-marches a voxelized surface point cloud (e.g. a converted 3DGS scene).

    MARSIM-style map rendering: instead of analytic primitives, the map is a
    boolean occupancy grid built from the point cloud (each occupied voxel
    dilated by one cell so marching steps cannot tunnel between neighbouring
    points), and a scan is the first range at which a ray's march enters an
    occupied voxel. This is an approximation (gaps show through at low point
    density), so keep `occupancy_radius` comparable to the source map's own
    point spacing, and `step` at or below `occupancy_radius`.
    """

    MAX_VOXELS = 200_000_000

    def __init__(self, points, occupancy_radius=0.15, step=0.1):
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("point cloud must be a nonempty (N,3) array")
        if not np.isfinite(points).all():
            raise ValueError("point cloud must be finite")
        if not np.isfinite([occupancy_radius, step]).all() or occupancy_radius <= 0 or step <= 0:
            raise ValueError("occupancy radius and step must be finite and positive")
        self.points = points
        self.occupancy_radius = float(occupancy_radius)
        self.step = float(step)
        self._voxel = self.occupancy_radius
        self._origin = points.min(axis=0) - 2 * self._voxel
        dims = np.ceil((points.max(axis=0) + 2 * self._voxel - self._origin) / self._voxel).astype(np.int64) + 1
        if np.prod(dims.astype(np.int64)) > self.MAX_VOXELS:
            raise ValueError("occupancy_radius too small for this point cloud's extent")
        self._dims = dims
        index = np.floor((points - self._origin) / self._voxel).astype(np.int64)
        grid = np.zeros(tuple(dims), dtype=bool)
        grid[index[:, 0], index[:, 1], index[:, 2]] = True
        self._grid = binary_dilation(grid, iterations=1)

    @classmethod
    def from_file(cls, path, **kwargs):
        return cls(load_xyz_ply(path), **kwargs)

    def point_cloud(self):
        """Surface samples for map display; scans do not cheat using this cloud."""
        return self.points

    def ranges(self, origin, directions, min_range, max_range):
        """March each ray outward and stop at the first occupied voxel."""
        origin, directions = _validate_rays(origin, directions, min_range, max_range)
        n_steps = int(np.ceil((max_range - min_range) / self.step)) + 1
        distances = np.full(len(directions), np.inf)
        unresolved = np.arange(len(directions))
        for depth in min_range + self.step * np.arange(n_steps):
            if unresolved.size == 0:
                break
            depth = min(depth, max_range)
            samples = origin + directions[unresolved] * depth
            index = np.floor((samples - self._origin) / self._voxel).astype(np.int64)
            in_bounds = ((index >= 0) & (index < self._dims)).all(axis=1)
            hit = np.zeros(len(samples), dtype=bool)
            bounded = index[in_bounds]
            hit[in_bounds] = self._grid[bounded[:, 0], bounded[:, 1], bounded[:, 2]]
            distances[unresolved[hit]] = depth
            unresolved = unresolved[~hit]
        distances[(distances < min_range) | (distances > max_range)] = np.inf
        return distances


def load_scene(data):
    """Pillars are the default/regression scene; point_cloud is experimental (see README)."""
    kind = data.get("type", "pillars")
    if kind == "pillars":
        return Scene.from_dict(data)
    if kind == "point_cloud":
        path = Path(data["path"])
        if not path.is_absolute():
            raise ValueError("point_cloud scene path must be absolute")
        return PointCloudScene.from_file(path, occupancy_radius=float(data.get("occupancy_radius_m", 0.15)),
                                         step=float(data.get("step_m", 0.2)))
    raise ValueError("scene type must be 'pillars' or 'point_cloud'")


def beam_directions(channels=16, columns=720, vertical_fov_deg=30.0):
    if not isinstance(channels, int) or not isinstance(columns, int) or not 1 <= channels <= 128 or not 4 <= columns <= 4096:
        raise ValueError("invalid beam grid")
    if not np.isfinite(vertical_fov_deg) or not 0 < vertical_fov_deg <= 180:
        raise ValueError("invalid vertical field of view")
    elevation = np.array([0.0]) if channels == 1 else np.linspace(-vertical_fov_deg / 2, vertical_fov_deg / 2, channels)
    elevation, azimuth = np.meshgrid(np.radians(elevation), np.linspace(-np.pi, np.pi, columns, endpoint=False), indexing="ij")
    return np.column_stack((np.cos(elevation).ravel() * np.cos(azimuth).ravel(),
                            np.cos(elevation).ravel() * np.sin(azimuth).ravel(), np.sin(elevation).ravel()))


def quaternion_rotation(x, y, z, w):
    q = np.asarray([x, y, z, w], dtype=float)
    if not np.isfinite(q).all() or abs(np.dot(q, q) - 1) > 1e-5:
        raise ValueError("odometry quaternion must be finite and unit length")
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def scan(scene, origin, rotation, directions, min_range, max_range, noise_std=0.0, seed=0):
    if not np.isfinite(rotation).all() or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(rotation), 1):
        raise ValueError("sensor rotation must be in SO(3)")
    if not np.isfinite(noise_std) or noise_std < 0:
        raise ValueError("range noise must be finite and non-negative")
    distances = scene.ranges(origin, directions @ rotation.T, min_range, max_range)
    hit = np.isfinite(distances)
    ranges = distances[hit]
    if noise_std:
        ranges = ranges + np.random.default_rng(seed).normal(0, noise_std, len(ranges))
    valid = (ranges >= min_range) & (ranges <= max_range)
    return (directions[hit][valid] * ranges[valid, None]).astype(np.float32)
