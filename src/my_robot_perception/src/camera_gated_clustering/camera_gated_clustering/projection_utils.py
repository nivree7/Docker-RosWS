#!/usr/bin/env python3
"""Projection utilities for 3D-to-2D camera mapping."""

import numpy as np
from typing import Tuple, Optional
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation


class ProjectionUtils:
    """Utilities for projecting 3D points to 2D image plane."""

    def __init__(self, camera_info=None):
        """Initialize projection utilities."""
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = None
        self.image_height = None
        
        if camera_info is not None:
            self.update_camera_info(camera_info)
    
    def update_camera_info(self, camera_info):
        """Update camera intrinsic parameters."""
        # Extract camera matrix (K)
        K = np.array(camera_info.k).reshape(3, 3)
        self.camera_matrix = K
        
        # Extract distortion coefficients
        self.dist_coeffs = np.array(camera_info.d) if len(camera_info.d) > 0 else None
        
        # Image dimensions
        self.image_width = camera_info.width
        self.image_height = camera_info.height
        
    def project_point_3d_to_2d(
        self, 
        point_3d: np.ndarray,
        check_bounds: bool = True,
        margin: int = 20
    ) -> Optional[Tuple[float, float]]:
        """Project a 3D point into 2D image coordinates."""
        if self.camera_matrix is None:
            raise ValueError("Camera intrinsics not set. Call update_camera_info() first.")
        
        # Check if point is in front of camera
        if point_3d[2] <= 0:
            return None
        
        # Project using pinhole camera model
        # [u, v, 1]^T = K * [x, y, z]^T / z
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        
        u = (fx * point_3d[0] / point_3d[2]) + cx
        v = (fy * point_3d[1] / point_3d[2]) + cy
        
        # Check bounds
        if check_bounds:
            if not self._is_within_bounds(u, v, margin):
                return None
        
        return (u, v)
    
    def project_points_batch(
        self,
        points_3d: np.ndarray,
        check_bounds: bool = True,
        margin: int = 20
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Project multiple 3D points to 2D image coordinates."""
        if self.camera_matrix is None:
            raise ValueError("Camera intrinsics not set.")
        
        # Filter points behind camera
        z_positive = points_3d[:, 2] > 0
        
        # Initialize outputs
        points_2d = np.zeros((len(points_3d), 2))
        valid_mask = np.zeros(len(points_3d), dtype=bool)
        
        if not np.any(z_positive):
            return points_2d, valid_mask
        
        # Project valid points
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        
        valid_points = points_3d[z_positive]
        u = (fx * valid_points[:, 0] / valid_points[:, 2]) + cx
        v = (fy * valid_points[:, 1] / valid_points[:, 2]) + cy
        
        points_2d[z_positive, 0] = u
        points_2d[z_positive, 1] = v
        
        # Check bounds
        if check_bounds:
            in_bounds = self._is_within_bounds_batch(u, v, margin)
            valid_mask[z_positive] = in_bounds
        else:
            valid_mask = z_positive
        
        return points_2d, valid_mask
    
    def _is_within_bounds(self, u: float, v: float, margin: int = 20) -> bool:
        """Check if a pixel coordinate is within image bounds."""
        if self.image_width is None or self.image_height is None:
            return True  # Can't check without image dimensions
        
        return (margin <= u < self.image_width - margin and
                margin <= v < self.image_height - margin)
    
    def _is_within_bounds_batch(self, u: np.ndarray, v: np.ndarray, margin: int = 20) -> np.ndarray:
        """Check if pixel coordinates are within image bounds."""
        if self.image_width is None or self.image_height is None:
            return np.ones(len(u), dtype=bool)
        
        return ((margin <= u) & (u < self.image_width - margin) &
                (margin <= v) & (v < self.image_height - margin))
    
    def transform_points_to_camera_frame(
        self,
        points_3d: np.ndarray,
        transform: TransformStamped
    ) -> np.ndarray:
        """Transform 3D points from LiDAR frame to camera frame."""
        # Extract translation
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        # Extract rotation (quaternion to rotation matrix)
        quat = transform.transform.rotation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        rotation_matrix = rotation.as_matrix()
        
        # Apply transformation: P_cam = R * P_lidar + t
        points_camera = (rotation_matrix @ points_3d.T).T + translation
        
        return points_camera
    
    def get_3d_bbox_corners(self, centroid: np.ndarray, dimensions: np.ndarray) -> np.ndarray:
        """Return the 8 corners of a 3D bounding box."""
        l, w, h = dimensions / 2.0
        
        # Define 8 corners relative to centroid
        corners = np.array([
            [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h],  # Bottom
            [-l, -w, h],  [l, -w, h],  [l, w, h],  [-l, w, h]    # Top
        ])
        
        return corners + centroid
    
    def project_3d_bbox_to_2d(
        self,
        centroid: np.ndarray,
        dimensions: np.ndarray
    ) -> Optional[Tuple[float, float, float, float]]:
        """Project a 3D bounding box to a 2D image box."""
        # Get 8 corners of the 3D bbox
        corners_3d = self.get_3d_bbox_corners(centroid, dimensions)
        
        # Project all corners to 2D
        corners_2d, valid_mask = self.project_points_batch(corners_3d, check_bounds=False)
        
        # If no corners are valid, return None
        if not np.any(valid_mask):
            return None
        
        # Get bounding box from valid projected corners
        valid_corners = corners_2d[valid_mask]
        xmin = np.min(valid_corners[:, 0])
        ymin = np.min(valid_corners[:, 1])
        xmax = np.max(valid_corners[:, 0])
        ymax = np.max(valid_corners[:, 1])
        
        return (xmin, ymin, xmax, ymax)
    
    @staticmethod
    def calculate_iou_2d(bbox1: Tuple[float, float, float, float],
                         bbox2: Tuple[float, float, float, float]) -> float:
        """Calculate Intersection over Union (IoU) for 2D bounding boxes."""
        x1_min, y1_min, x1_max, y1_max = bbox1
        x2_min, y2_min, x2_max, y2_max = bbox2
        
        # Calculate intersection
        inter_xmin = max(x1_min, x2_min)
        inter_ymin = max(y1_min, y2_min)
        inter_xmax = min(x1_max, x2_max)
        inter_ymax = min(y1_max, y2_max)
        
        # Check if there's no intersection
        if inter_xmin >= inter_xmax or inter_ymin >= inter_ymax:
            return 0.0
        
        inter_area = (inter_xmax - inter_xmin) * (inter_ymax - inter_ymin)
        
        # Calculate union
        area1 = (x1_max - x1_min) * (y1_max - y1_min)
        area2 = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = area1 + area2 - inter_area
        
        # Avoid division by zero
        if union_area <= 0:
            return 0.0
        
        return inter_area / union_area
