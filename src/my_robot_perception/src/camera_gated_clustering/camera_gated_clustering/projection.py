"""Projection helpers for LiDAR-camera fusion."""

from __future__ import annotations

import numpy as np


def project_points(points_cam: np.ndarray, camera_matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Project 3D camera-frame points into image pixels.

    Args:
        points_cam: Array of shape (N, 3) in camera optical frame.
        camera_matrix: Intrinsic matrix (3x3).

    Returns:
        Tuple of (pixels Nx2, valid_mask). The mask is True where Z > 0.
    """
    if points_cam.size == 0:
        return np.empty((0, 2), dtype=np.float32), np.empty((0,), dtype=bool)

    z = points_cam[:, 2]
    valid = z > 0.0
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    u = fx * points_cam[:, 0] / z + cx
    v = fy * points_cam[:, 1] / z + cy
    pixels = np.stack([u, v], axis=1)
    return pixels.astype(np.float32), valid
