"""LiDAR geometry contracts run without ROS, Docker, or an aircraft."""

import sys

import numpy as np
import pytest

from tooling.env import get_git_root

sys.path.insert(0, str(get_git_root() / "infra/sim_infra/lidar_sim"))
from lidar_sim.geometry import (
    PointCloudScene,
    Pillar,
    Scene,
    beam_directions,
    load_scene,
    load_xyz_ply,
    quaternion_rotation,
    scan,
)


def test_nearest_surface_occludes_far_pillar():
    scene = Scene((Pillar(3, 0, 1, 4), Pillar(7, 0, 1, 4)))
    np.testing.assert_allclose(scene.ranges([0, 0, 2], [[1, 0, 0]], 0.3, 50), [2])


def test_blind_zone_does_not_see_through_surface():
    scene = Scene((Pillar(0.2, 0, 0.1, 4), Pillar(4, 0, 1, 4)))
    assert np.isinf(scene.ranges([0, 0, 2], [[1, 0, 0]], 0.3, 50)[0])


def test_caps_ground_and_bounded_map():
    scene = Scene((Pillar(3, 0, 1, 4),), half_extent=10)
    np.testing.assert_allclose(scene.ranges([3, 0, 6], [[0, 0, -1]], 0.3, 50), [2])
    np.testing.assert_allclose(scene.ranges([0, 0, 2], [[0, 0, -1]], 0.3, 50), [2])
    assert np.isinf(scene.ranges([30, 0, 2], [[0, 0, -1]], 0.3, 50)[0])
    assert np.isinf(scene.ranges([0, 0, 6], [[1, 0, 0]], 0.3, 50)[0])


def test_full_pose_rotation_and_sensor_frame_return():
    scene = Scene((Pillar(0, 3, 1, 4),))
    rotation = quaternion_rotation(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    points = scan(scene, [0, 0, 2], rotation, np.array([[1, 0, 0]]), 0.3, 50)
    np.testing.assert_allclose(points, [[2, 0, 0]], atol=1e-6)


def test_snapshot_noise_is_reproducible():
    scene = Scene((Pillar(3, 0, 1, 4),))
    args = (scene, [0, 0, 2], np.eye(3), np.array([[1, 0, 0]]), 0.3, 50)
    first = scan(*args, noise_std=0.01, seed=[7, 100, 0])
    np.testing.assert_array_equal(first, scan(*args, noise_std=0.01, seed=[7, 100, 0]))
    assert not np.array_equal(first, scan(*args, noise_std=0.01, seed=[7, 200, 0]))


def test_beam_grid_and_map_have_finite_samples():
    scene = Scene((Pillar(3, 0, 1, 4),), half_extent=8)
    directions = beam_directions()
    assert directions.shape == (16 * 720, 3)
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1)
    points = scene.point_cloud()
    assert points.shape[1] == 3 and points.dtype == np.float32
    assert np.isfinite(points).all()
    assert np.max(points[:, 2]) == 4


def test_full_sphere_fov_is_supported_for_omnidirectional_sensors():
    directions = beam_directions(channels=128, columns=360, vertical_fov_deg=180.0)
    assert directions.shape == (128 * 360, 3)
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1)
    assert directions[:, 2].min() < -0.99 and directions[:, 2].max() > 0.99


@pytest.mark.parametrize("q", [(0, 0, 0, 0), (0, 0, float("nan"), 1)])
def test_invalid_pose_is_rejected(q):
    with pytest.raises(ValueError):
        quaternion_rotation(*q)


@pytest.mark.parametrize("args", [(0, 720, 30), (16, 1, 30), (16, 720, 181)])
def test_invalid_grid_is_rejected(args):
    with pytest.raises(ValueError):
        beam_directions(*args)


def test_ranges_and_allocation_are_bounded():
    scene = Scene(())
    with pytest.raises(ValueError):
        scene.ranges([0, 0, 2], [[2, 0, 0]], 0.3, 50)
    with pytest.raises(ValueError):
        Scene((), map_resolution=0.00001)
    with pytest.raises(ValueError):
        Pillar(0, 0, -1, 3)
    with pytest.raises(ValueError):
        scan(scene, [0, 0, 2], np.eye(3), np.array([[1, 0, 0]]), 0.3, 50, noise_std=-1)


def test_no_hit_is_an_empty_dense_cloud():
    points = scan(Scene(()), [0, 0, 2], np.eye(3), np.array([[0, 0, 1]]), 0.3, 50)
    assert points.shape == (0, 3)


def _write_ply(path, points, fmt="binary_little_endian"):
    header = (f"ply\nformat {fmt} 1.0\nelement vertex {len(points)}\n"
              "property float x\nproperty float y\nproperty float z\nend_header\n")
    if fmt == "ascii":
        path.write_text(header + "\n".join(f"{x} {y} {z}" for x, y, z in points) + "\n")
    else:
        path.write_bytes(header.encode() + np.asarray(points, dtype="<f4").tobytes())


def test_ply_loader_round_trips_ascii_and_binary(tmp_path):
    points = np.array([[0, 0, 0], [1, 2, 3], [-4, 5, -6]], dtype=np.float32)
    binary, ascii_ = tmp_path / "pts.ply", tmp_path / "pts_ascii.ply"
    _write_ply(binary, points)
    _write_ply(ascii_, points, fmt="ascii")
    np.testing.assert_allclose(load_xyz_ply(binary), points)
    np.testing.assert_allclose(load_xyz_ply(ascii_), points)


def test_ply_loader_rejects_mesh_and_missing_fields(tmp_path):
    with_face = tmp_path / "mesh.ply"
    with_face.write_text("ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\n"
                         "property float y\nproperty float z\nelement face 0\n"
                         "property list uchar int vertex_indices\nend_header\n0 0 0\n")
    with pytest.raises(ValueError):
        load_xyz_ply(with_face)
    no_xyz = tmp_path / "nofields.ply"
    no_xyz.write_text("ply\nformat ascii 1.0\nelement vertex 1\nproperty float intensity\nend_header\n1.0\n")
    with pytest.raises(ValueError):
        load_xyz_ply(no_xyz)


def test_point_cloud_scene_first_return_and_occlusion():
    grid = np.linspace(-5, 5, 101)
    gx, gy = np.meshgrid(grid, grid)
    plane = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)]).astype(np.float32)
    scene = PointCloudScene(plane, occupancy_radius=0.15, step=0.1)
    down = scene.ranges([0, 0, 5], np.array([[0, 0, -1]]), 0.3, 50)
    assert abs(down[0] - 5) < 0.3
    sideways = scene.ranges([0, 0, 5], np.array([[1, 0, 0]]), 0.3, 50)
    assert np.isinf(sideways[0])


def test_point_cloud_scene_rejects_invalid_inputs():
    with pytest.raises(ValueError):
        PointCloudScene(np.zeros((0, 3)))
    with pytest.raises(ValueError):
        PointCloudScene(np.zeros((4, 3)), occupancy_radius=-1)
    scene = PointCloudScene(np.array([[0.0, 0.0, 0.0]]))
    with pytest.raises(ValueError):
        scene.ranges([0, 0, 0], [[2, 0, 0]], 0.3, 50)


def test_load_scene_dispatches_on_type(tmp_path):
    pillars = load_scene({"pillars": [{"x": 0, "y": 0, "radius": 1, "height": 4}]})
    assert isinstance(pillars, Scene)
    ply = tmp_path / "scene.ply"
    _write_ply(ply, np.array([[0, 0, 0], [1, 1, 0]], dtype=np.float32))
    cloud = load_scene({"type": "point_cloud", "path": str(ply)})
    assert isinstance(cloud, PointCloudScene)
    with pytest.raises(ValueError):
        load_scene({"type": "point_cloud", "path": "relative.ply"})
    with pytest.raises(ValueError):
        load_scene({"type": "nonsense"})
