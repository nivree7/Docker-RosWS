#!/usr/bin/env python3
"""Cluster association algorithms for LiDAR-camera matching."""

import numpy as np
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass


@dataclass
class LiDARCluster:
    """Represent a LiDAR cluster."""
    cluster_id: int
    centroid: np.ndarray  # [x, y, z] in camera frame
    dimensions: np.ndarray  # [length, width, height]
    bbox_2d: Optional[Tuple[float, float, float, float]]  # (xmin, ymin, xmax, ymax)
    point_count: int
    confidence: float


@dataclass
class CameraDetection:
    """Represent a camera object detection."""
    detection_id: int
    bbox_2d: Tuple[float, float, float, float]  # (xmin, ymin, xmax, ymax)
    object_class: str
    confidence: float
    position_3d: Optional[np.ndarray] = None  # [x, y, z] if available from ZED


@dataclass
class Association:
    """Represent a cluster-detection association."""
    lidar_cluster_id: int
    camera_detection_id: int
    iou_score: float
    centroid_distance_px: float
    spatial_distance_3d: float
    combined_score: float
    object_class: str
    camera_confidence: float


class ClusterAssociator:
    """Associate LiDAR clusters with camera detections."""

    def __init__(self, config: dict):
        """Initialize associator with configuration."""
        self.config = config
        self.method = config.get('method', 'hybrid')
        self.iou_threshold = config.get('iou_threshold', 0.25)
        self.max_centroid_distance_px = config.get('max_centroid_distance_px', 100)
        self.max_spatial_distance_m = config.get('max_spatial_distance_m', 2.5)
        
        # Hybrid method weights
        self.weight_iou = config.get('weight_iou', 0.4)
        self.weight_centroid = config.get('weight_centroid', 0.3)
        self.weight_spatial = config.get('weight_spatial', 0.3)
        self.min_hybrid_score = config.get('min_hybrid_score', 0.5)
    
    def associate(
        self,
        lidar_clusters: List[LiDARCluster],
        camera_detections: List[CameraDetection],
        projection_utils
    ) -> Tuple[List[Association], List[int], List[int]]:
        """Associate LiDAR clusters with camera detections."""
        if self.method == 'iou':
            return self._associate_iou(lidar_clusters, camera_detections, projection_utils)
        elif self.method == 'centroid':
            return self._associate_centroid(lidar_clusters, camera_detections, projection_utils)
        elif self.method == 'distance_3d':
            return self._associate_distance_3d(lidar_clusters, camera_detections)
        elif self.method == 'hybrid':
            return self._associate_hybrid(lidar_clusters, camera_detections, projection_utils)
        else:
            raise ValueError(f"Unknown association method: {self.method}")
    
    def _associate_iou(
        self,
        lidar_clusters: List[LiDARCluster],
        camera_detections: List[CameraDetection],
        projection_utils
    ) -> Tuple[List[Association], List[int], List[int]]:
        """Associate using 2D bounding box IoU."""
        associations = []
        matched_lidar = set()
        matched_camera = set()
        
        # Build cost matrix
        cost_matrix = np.zeros((len(lidar_clusters), len(camera_detections)))
        
        for i, cluster in enumerate(lidar_clusters):
            if cluster.bbox_2d is None:
                continue
                
            for j, detection in enumerate(camera_detections):
                iou = projection_utils.calculate_iou_2d(cluster.bbox_2d, detection.bbox_2d)
                cost_matrix[i, j] = iou
        
        # Greedy matching (highest IoU first)
        while True:
            if cost_matrix.size == 0 or np.max(cost_matrix) < self.iou_threshold:
                break
            
            # Find best match
            i, j = np.unravel_index(np.argmax(cost_matrix), cost_matrix.shape)
            iou = cost_matrix[i, j]
            
            if iou < self.iou_threshold:
                break
            
            cluster = lidar_clusters[i]
            detection = camera_detections[j]
            
            # Create association
            association = Association(
                lidar_cluster_id=cluster.cluster_id,
                camera_detection_id=detection.detection_id,
                iou_score=iou,
                centroid_distance_px=0.0,  # Not computed in this method
                spatial_distance_3d=0.0,   # Not computed in this method
                combined_score=iou,
                object_class=detection.object_class,
                camera_confidence=detection.confidence
            )
            associations.append(association)
            
            matched_lidar.add(cluster.cluster_id)
            matched_camera.add(detection.detection_id)
            
            # Remove matched pair from consideration
            cost_matrix[i, :] = 0
            cost_matrix[:, j] = 0
        
        # Find unmatched
        unmatched_lidar = [c.cluster_id for c in lidar_clusters if c.cluster_id not in matched_lidar]
        unmatched_camera = [d.detection_id for d in camera_detections if d.detection_id not in matched_camera]
        
        return associations, unmatched_lidar, unmatched_camera
    
    def _associate_centroid(
        self,
        lidar_clusters: List[LiDARCluster],
        camera_detections: List[CameraDetection],
        projection_utils
    ) -> Tuple[List[Association], List[int], List[int]]:
        """Associate using centroid distance in image space."""
        associations = []
        matched_lidar = set()
        matched_camera = set()
        
        # Project cluster centroids to 2D
        cluster_centroids_2d = []
        for cluster in lidar_clusters:
            centroid_2d = projection_utils.project_point_3d_to_2d(
                cluster.centroid,
                check_bounds=False
            )
            cluster_centroids_2d.append(centroid_2d)
        
        # Calculate detection centroids
        detection_centroids_2d = []
        for detection in camera_detections:
            xmin, ymin, xmax, ymax = detection.bbox_2d
            cx = (xmin + xmax) / 2
            cy = (ymin + ymax) / 2
            detection_centroids_2d.append((cx, cy))
        
        # Build distance matrix
        distance_matrix = np.full((len(lidar_clusters), len(camera_detections)), np.inf)
        
        for i, centroid_2d in enumerate(cluster_centroids_2d):
            if centroid_2d is None:
                continue
            
            for j, det_centroid in enumerate(detection_centroids_2d):
                dist = np.sqrt((centroid_2d[0] - det_centroid[0])**2 + 
                             (centroid_2d[1] - det_centroid[1])**2)
                distance_matrix[i, j] = dist
        
        # Greedy matching (smallest distance first)
        while True:
            if distance_matrix.size == 0 or np.min(distance_matrix) > self.max_centroid_distance_px:
                break
            
            i, j = np.unravel_index(np.argmin(distance_matrix), distance_matrix.shape)
            dist = distance_matrix[i, j]
            
            if dist > self.max_centroid_distance_px:
                break
            
            cluster = lidar_clusters[i]
            detection = camera_detections[j]
            
            # Normalized score (closer = better)
            score = 1.0 - (dist / self.max_centroid_distance_px)
            
            association = Association(
                lidar_cluster_id=cluster.cluster_id,
                camera_detection_id=detection.detection_id,
                iou_score=0.0,
                centroid_distance_px=dist,
                spatial_distance_3d=0.0,
                combined_score=score,
                object_class=detection.object_class,
                camera_confidence=detection.confidence
            )
            associations.append(association)
            
            matched_lidar.add(cluster.cluster_id)
            matched_camera.add(detection.detection_id)
            
            distance_matrix[i, :] = np.inf
            distance_matrix[:, j] = np.inf
        
        unmatched_lidar = [c.cluster_id for c in lidar_clusters if c.cluster_id not in matched_lidar]
        unmatched_camera = [d.detection_id for d in camera_detections if d.detection_id not in matched_camera]
        
        return associations, unmatched_lidar, unmatched_camera
    
    def _associate_distance_3d(
        self,
        lidar_clusters: List[LiDARCluster],
        camera_detections: List[CameraDetection]
    ) -> Tuple[List[Association], List[int], List[int]]:
        """Associate using 3D spatial distance."""
        associations = []
        matched_lidar = set()
        matched_camera = set()
        
        # Build distance matrix
        distance_matrix = np.full((len(lidar_clusters), len(camera_detections)), np.inf)
        
        for i, cluster in enumerate(lidar_clusters):
            for j, detection in enumerate(camera_detections):
                if detection.position_3d is not None:
                    dist = np.linalg.norm(cluster.centroid - detection.position_3d)
                    distance_matrix[i, j] = dist
        
        # Greedy matching
        while True:
            if distance_matrix.size == 0 or np.min(distance_matrix) > self.max_spatial_distance_m:
                break
            
            i, j = np.unravel_index(np.argmin(distance_matrix), distance_matrix.shape)
            dist = distance_matrix[i, j]
            
            if dist > self.max_spatial_distance_m:
                break
            
            cluster = lidar_clusters[i]
            detection = camera_detections[j]
            
            score = 1.0 - (dist / self.max_spatial_distance_m)
            
            association = Association(
                lidar_cluster_id=cluster.cluster_id,
                camera_detection_id=detection.detection_id,
                iou_score=0.0,
                centroid_distance_px=0.0,
                spatial_distance_3d=dist,
                combined_score=score,
                object_class=detection.object_class,
                camera_confidence=detection.confidence
            )
            associations.append(association)
            
            matched_lidar.add(cluster.cluster_id)
            matched_camera.add(detection.detection_id)
            
            distance_matrix[i, :] = np.inf
            distance_matrix[:, j] = np.inf
        
        unmatched_lidar = [c.cluster_id for c in lidar_clusters if c.cluster_id not in matched_lidar]
        unmatched_camera = [d.detection_id for d in camera_detections if d.detection_id not in matched_camera]
        
        return associations, unmatched_lidar, unmatched_camera
    
    def _associate_hybrid(
        self,
        lidar_clusters: List[LiDARCluster],
        camera_detections: List[CameraDetection],
        projection_utils
    ) -> Tuple[List[Association], List[int], List[int]]:
        """Associate using a weighted combination of multiple metrics."""
        associations = []
        matched_lidar = set()
        matched_camera = set()
        
        # Precompute all metrics
        cluster_centroids_2d = [
            projection_utils.project_point_3d_to_2d(c.centroid, check_bounds=False)
            for c in lidar_clusters
        ]
        
        detection_centroids_2d = [
            ((d.bbox_2d[0] + d.bbox_2d[2]) / 2, (d.bbox_2d[1] + d.bbox_2d[3]) / 2)
            for d in camera_detections
        ]
        
        # Build comprehensive score matrix
        score_matrix = np.zeros((len(lidar_clusters), len(camera_detections)))
        
        for i, cluster in enumerate(lidar_clusters):
            for j, detection in enumerate(camera_detections):
                # IoU score
                iou_score = 0.0
                if cluster.bbox_2d is not None:
                    iou_score = projection_utils.calculate_iou_2d(cluster.bbox_2d, detection.bbox_2d)
                
                # Centroid distance score
                centroid_score = 0.0
                if cluster_centroids_2d[i] is not None:
                    dist_px = np.sqrt(
                        (cluster_centroids_2d[i][0] - detection_centroids_2d[j][0])**2 +
                        (cluster_centroids_2d[i][1] - detection_centroids_2d[j][1])**2
                    )
                    if dist_px < self.max_centroid_distance_px:
                        centroid_score = 1.0 - (dist_px / self.max_centroid_distance_px)
                
                # 3D distance score
                spatial_score = 0.0
                if detection.position_3d is not None:
                    dist_3d = np.linalg.norm(cluster.centroid - detection.position_3d)
                    if dist_3d < self.max_spatial_distance_m:
                        spatial_score = 1.0 - (dist_3d / self.max_spatial_distance_m)
                
                # Combined score
                combined = (self.weight_iou * iou_score +
                           self.weight_centroid * centroid_score +
                           self.weight_spatial * spatial_score)
                
                score_matrix[i, j] = combined
        
        # Greedy matching
        while True:
            if score_matrix.size == 0 or np.max(score_matrix) < self.min_hybrid_score:
                break
            
            i, j = np.unravel_index(np.argmax(score_matrix), score_matrix.shape)
            score = score_matrix[i, j]
            
            if score < self.min_hybrid_score:
                break
            
            cluster = lidar_clusters[i]
            detection = camera_detections[j]
            
            # Compute individual metrics for logging
            iou = 0.0
            if cluster.bbox_2d is not None:
                iou = projection_utils.calculate_iou_2d(cluster.bbox_2d, detection.bbox_2d)
            
            centroid_dist = 0.0
            if cluster_centroids_2d[i] is not None:
                centroid_dist = np.sqrt(
                    (cluster_centroids_2d[i][0] - detection_centroids_2d[j][0])**2 +
                    (cluster_centroids_2d[i][1] - detection_centroids_2d[j][1])**2
                )
            
            spatial_dist = 0.0
            if detection.position_3d is not None:
                spatial_dist = np.linalg.norm(cluster.centroid - detection.position_3d)
            
            association = Association(
                lidar_cluster_id=cluster.cluster_id,
                camera_detection_id=detection.detection_id,
                iou_score=iou,
                centroid_distance_px=centroid_dist,
                spatial_distance_3d=spatial_dist,
                combined_score=score,
                object_class=detection.object_class,
                camera_confidence=detection.confidence
            )
            associations.append(association)
            
            matched_lidar.add(cluster.cluster_id)
            matched_camera.add(detection.detection_id)
            
            score_matrix[i, :] = 0
            score_matrix[:, j] = 0
        
        unmatched_lidar = [c.cluster_id for c in lidar_clusters if c.cluster_id not in matched_lidar]
        unmatched_camera = [d.detection_id for d in camera_detections if d.detection_id not in matched_camera]
        
        return associations, unmatched_lidar, unmatched_camera
