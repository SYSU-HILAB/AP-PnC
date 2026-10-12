"""Deterministic point-cloud map and analytic first-return cylinder ray casting."""

from dataclasses import dataclass

import numpy as np


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


def beam_directions(channels=16, columns=720, vertical_fov_deg=30.0):
    if not isinstance(channels, int) or not isinstance(columns, int) or not 1 <= channels <= 128 or not 4 <= columns <= 4096:
        raise ValueError("invalid beam grid")
    if not np.isfinite(vertical_fov_deg) or not 0 < vertical_fov_deg < 180:
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
