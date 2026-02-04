"""Unit tests for projection helpers."""

import numpy as np

from camera_gated_clustering.projection import project_points


def test_project_points_center():
    camera_matrix = np.array(
        [
            [100.0, 0.0, 320.0],
            [0.0, 100.0, 240.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    points = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)

    pixels, mask = project_points(points, camera_matrix)

    assert mask.tolist() == [True]
    assert np.allclose(pixels[0], [320.0, 240.0])


def test_project_points_negative_depth():
    camera_matrix = np.eye(3, dtype=np.float32)
    points = np.array([[1.0, 1.0, -1.0]], dtype=np.float32)

    pixels, mask = project_points(points, camera_matrix)

    assert mask.tolist() == [False]
    assert pixels.shape == (1, 2)
