#!/usr/bin/env python3
"""Gating strategies for LiDAR-camera fusion."""

from typing import List, Dict
from enum import Enum
import numpy as np


class GatingMode(Enum):
    """Available gating modes."""
    HARD = "hard"           # Only keep camera-validated clusters
    SOFT = "soft"           # Keep all, add camera labels when available
    CONFIDENCE = "confidence"  # Gate based on combined confidence
    DISTANCE_ADAPTIVE = "distance_adaptive"  # Adapt based on distance


class GatingStrategy:
    """Base class for gating strategies."""

    def __init__(self, config: dict):
        """Initialize the gating strategy."""
        self.config = config
        self.min_camera_confidence = config.get('min_camera_confidence', 0.45)
        self.min_lidar_confidence = config.get('min_lidar_confidence', 0.35)
        self.camera_weight = config.get('camera_weight', 0.6)
        self.lidar_weight = config.get('lidar_weight', 0.4)
    
    def apply(self, clusters: List, associations: List, unmatched_lidar: List) -> List:
        """Apply gating to clusters."""
        raise NotImplementedError("Subclasses must implement apply()")
    
    def calculate_fusion_confidence(
        self,
        lidar_confidence: float,
        camera_confidence: float,
        camera_validated: bool
    ) -> float:
        """Calculate combined fusion confidence."""
        if not camera_validated:
            return lidar_confidence
        
        # Weighted average
        fusion_conf = (self.lidar_weight * lidar_confidence + 
                      self.camera_weight * camera_confidence)
        
        return min(1.0, fusion_conf)
    
    def calculate_cluster_confidence(
        self,
        point_count: int,
        average_distance: float,
        density: float = None
    ) -> float:
        """Calculate confidence score for a LiDAR cluster."""
        min_points = self.config.get('min_cluster_points', 30)
        max_points = self.config.get('max_cluster_points', 15000)
        preferred_density = self.config.get('preferred_density', 0.5)
        
        # Point count score (prefer clusters with reasonable point counts)
        if point_count < min_points:
            count_score = point_count / min_points
        elif point_count > max_points:
            count_score = max(0.5, 1.0 - (point_count - max_points) / max_points)
        else:
            count_score = 1.0
        
        # Distance score (closer objects are more reliable)
        max_dist = 20.0
        distance_score = max(0.0, 1.0 - (average_distance / max_dist))
        
        # Density score (if available)
        density_score = 1.0
        if density is not None:
            density_score = min(1.0, density / preferred_density)
        
        # Combined confidence
        confidence = 0.4 * count_score + 0.4 * distance_score + 0.2 * density_score
        
        return min(1.0, max(0.0, confidence))


class HardGating(GatingStrategy):
    """Only keep clusters validated by camera."""
    
    def apply(self, clusters: List, associations: List, unmatched_lidar: List) -> List:
        """Keep only camera-validated clusters."""
        validated_ids = {assoc.lidar_cluster_id for assoc in associations}
        
        filtered_clusters = []
        for cluster in clusters:
            if cluster.cluster_id in validated_ids:
                # Additional check: camera confidence must meet minimum
                if cluster.camera_confidence >= self.min_camera_confidence:
                    filtered_clusters.append(cluster)
        
        return filtered_clusters


class SoftGating(GatingStrategy):
    """Keep all clusters, add camera labels when available."""
    
    def apply(self, clusters: List, associations: List, unmatched_lidar: List) -> List:
        """Keep all clusters, enrich with camera data where possible."""
        # All clusters pass through
        # Camera labels are already added during association
        
        # Optionally filter by minimum LiDAR confidence
        filtered_clusters = []
        for cluster in clusters:
            # Calculate fusion confidence
            cluster.fusion_confidence = self.calculate_fusion_confidence(
                cluster.lidar_confidence,
                cluster.camera_confidence,
                cluster.camera_validated
            )
            
            # Keep all or apply minimum threshold
            if cluster.lidar_confidence >= self.min_lidar_confidence:
                filtered_clusters.append(cluster)
        
        return filtered_clusters


class ConfidenceGating(GatingStrategy):
    """Gate based on combined confidence score."""
    
    def __init__(self, config: dict):
        super().__init__(config)
        self.min_fusion_confidence = config.get('min_fusion_confidence', 0.5)
    
    def apply(self, clusters: List, associations: List, unmatched_lidar: List) -> List:
        """Keep clusters with fusion confidence above threshold."""
        filtered_clusters = []
        
        for cluster in clusters:
            # Calculate fusion confidence
            cluster.fusion_confidence = self.calculate_fusion_confidence(
                cluster.lidar_confidence,
                cluster.camera_confidence,
                cluster.camera_validated
            )
            
            # Apply threshold
            if cluster.fusion_confidence >= self.min_fusion_confidence:
                filtered_clusters.append(cluster)
        
        return filtered_clusters


class DistanceAdaptiveGating(GatingStrategy):
    """Adapt gating strategy based on distance from sensor."""
    
    def __init__(self, config: dict):
        super().__init__(config)
        self.near_threshold = config.get('near_threshold_m', 5.0)
        self.far_threshold = config.get('far_threshold_m', 15.0)
    
    def apply(self, clusters: List, associations: List, unmatched_lidar: List) -> List:
        """Apply distance-adaptive gating."""
        filtered_clusters = []
        
        for cluster in clusters:
            distance = cluster.average_distance
            
            # Near objects: require camera validation (camera is more reliable)
            if distance < self.near_threshold:
                if cluster.camera_validated and cluster.camera_confidence >= self.min_camera_confidence:
                    cluster.fusion_confidence = self.calculate_fusion_confidence(
                        cluster.lidar_confidence,
                        cluster.camera_confidence,
                        True
                    )
                    filtered_clusters.append(cluster)
            
            # Medium distance: use hybrid approach
            elif distance < self.far_threshold:
                cluster.fusion_confidence = self.calculate_fusion_confidence(
                    cluster.lidar_confidence,
                    cluster.camera_confidence if cluster.camera_validated else 0.0,
                    cluster.camera_validated
                )
                if cluster.fusion_confidence >= 0.4:
                    filtered_clusters.append(cluster)
            
            # Far objects: rely on LiDAR (camera less reliable)
            else:
                if cluster.lidar_confidence >= self.min_lidar_confidence:
                    cluster.fusion_confidence = cluster.lidar_confidence
                    filtered_clusters.append(cluster)
        
        return filtered_clusters


class GatingFactory:
    """Factory for creating gating strategies."""
    
    @staticmethod
    def create(mode: str, config: dict) -> GatingStrategy:
        """Create a gating strategy based on mode."""
        mode_map = {
            'hard': HardGating,
            'soft': SoftGating,
            'confidence': ConfidenceGating,
            'distance_adaptive': DistanceAdaptiveGating
        }
        
        strategy_class = mode_map.get(mode.lower())
        if strategy_class is None:
            raise ValueError(f"Unknown gating mode: {mode}. "
                           f"Available: {list(mode_map.keys())}")
        
        return strategy_class(config)
