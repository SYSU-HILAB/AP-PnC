"""LiDAR geometry contracts run without ROS, Docker, or an aircraft."""

import sys

import numpy as np
import pytest

from tooling.env import get_git_root

sys.path.insert(0, str(get_git_root() / "infra/sim_infra/lidar_sim"))
from lidar_sim.geometry import Pillar, Scene, beam_directions, quaternion_rotation, scan


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


@pytest.mark.parametrize("q", [(0, 0, 0, 0), (0, 0, float("nan"), 1)])
def test_invalid_pose_is_rejected(q):
    with pytest.raises(ValueError):
        quaternion_rotation(*q)


@pytest.mark.parametrize("args", [(0, 720, 30), (16, 1, 30), (16, 720, 180)])
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
