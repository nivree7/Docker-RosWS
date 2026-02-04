#!/usr/bin/env python3
"""Provide visualization helpers for sensor fusion."""

import cv2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import cv_bridge


class FusionVisualizer:
    """Handles visualization of fusion results."""

    def __init__(self, config: dict):
        """Initialize the visualizer with configuration."""
        self.config = config
        self.marker_lifetime_ms = config.get('marker_lifetime_ms', 200)

        # Color scheme
        self.color_lidar_only = config.get('color_lidar_only', [0.0, 1.0, 0.5])
        self.color_camera_validated = config.get('color_camera_validated', [1.0, 0.8, 0.0])
        self.color_camera_only = config.get('color_camera_only', [0.0, 0.5, 1.0])

        self.bridge = cv_bridge.CvBridge()

    def create_marker_array(self, enriched_clusters, header) -> MarkerArray:
        """Create a MarkerArray for enriched clusters."""
        marker_array = MarkerArray()

        for cluster in enriched_clusters:
            bbox_marker = self._create_bbox_marker(cluster, header)
            marker_array.markers.append(bbox_marker)

            label_marker = self._create_label_marker(cluster, header)
            marker_array.markers.append(label_marker)

            centroid_marker = self._create_centroid_marker(cluster, header)
            marker_array.markers.append(centroid_marker)

        return marker_array

    def _create_bbox_marker(self, cluster, header) -> Marker:
        """Create 3D bounding box marker."""
        marker = Marker()
        marker.header = header
        marker.ns = "cluster_bboxes"
        marker.id = cluster.cluster_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03

        if cluster.camera_validated:
            color = self.color_camera_validated
        else:
            color = self.color_lidar_only

        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
        marker.lifetime.nanosec = self.marker_lifetime_ms * 1_000_000

        cx, cy, cz = cluster.centroid.x, cluster.centroid.y, cluster.centroid.z
        l, w, h = (
            cluster.dimensions.x / 2,
            cluster.dimensions.y / 2,
            cluster.dimensions.z / 2,
        )

        corners = [
            Point(x=cx - l, y=cy - w, z=cz - h),
            Point(x=cx + l, y=cy - w, z=cz - h),
            Point(x=cx + l, y=cy + w, z=cz - h),
            Point(x=cx - l, y=cy + w, z=cz - h),
            Point(x=cx - l, y=cy - w, z=cz + h),
            Point(x=cx + l, y=cy - w, z=cz + h),
            Point(x=cx + l, y=cy + w, z=cz + h),
            Point(x=cx - l, y=cy + w, z=cz + h),
        ]

        marker.points.extend([
            corners[0], corners[1], corners[1], corners[2],
            corners[2], corners[3], corners[3], corners[0]
        ])
        marker.points.extend([
            corners[4], corners[5], corners[5], corners[6],
            corners[6], corners[7], corners[7], corners[4]
        ])
        marker.points.extend([
            corners[0], corners[4], corners[1], corners[5],
            corners[2], corners[6], corners[3], corners[7]
        ])

        return marker

    def _create_label_marker(self, cluster, header) -> Marker:
        """Create text label marker."""
        marker = Marker()
        marker.header = header
        marker.ns = "cluster_labels"
        marker.id = cluster.cluster_id + 10000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = cluster.centroid.x
        marker.pose.position.y = cluster.centroid.y
        marker.pose.position.z = cluster.centroid.z + cluster.dimensions.z / 2 + 0.3
        marker.pose.orientation.w = 1.0

        if cluster.camera_validated:
            marker.text = f"{cluster.object_class}\n{cluster.fusion_confidence:.2f}"
        else:
            marker.text = f"Cluster {cluster.cluster_id}\n{cluster.lidar_confidence:.2f}"

        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.lifetime.nanosec = self.marker_lifetime_ms * 1_000_000

        return marker

    def _create_centroid_marker(self, cluster, header) -> Marker:
        """Create centroid point marker."""
        marker = Marker()
        marker.header = header
        marker.ns = "cluster_centroids"
        marker.id = cluster.cluster_id + 20000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = cluster.centroid
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        if cluster.camera_validated:
            color = self.color_camera_validated
        else:
            color = self.color_lidar_only

        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.8)
        marker.lifetime.nanosec = self.marker_lifetime_ms * 1_000_000

        return marker

    def create_debug_image(
        self,
        camera_image,
        lidar_clusters,
        camera_detections,
        associations,
        projection_utils,
    ):
        """Create a debug image with overlays."""
        debug_img = camera_image.copy()

        for detection in camera_detections:
            xmin, ymin, xmax, ymax = [int(x) for x in detection.bbox_2d]
            cv2.rectangle(debug_img, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)

            label = f"{detection.object_class}: {detection.confidence:.2f}"
            cv2.putText(
                debug_img,
                label,
                (xmin, ymin - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2,
            )

        associated_ids = {a.lidar_cluster_id for a in associations}

        for cluster in lidar_clusters:
            if cluster.bbox_2d is None:
                centroid_2d = projection_utils.project_point_3d_to_2d(
                    cluster.centroid, check_bounds=False
                )
                if centroid_2d is not None:
                    cx, cy = int(centroid_2d[0]), int(centroid_2d[1])
                    color = (0, 255, 0) if cluster.cluster_id in associated_ids else (0, 0, 255)
                    cv2.circle(debug_img, (cx, cy), 5, color, -1)
            else:
                xmin, ymin, xmax, ymax = [int(x) for x in cluster.bbox_2d]
                color = (0, 255, 0) if cluster.cluster_id in associated_ids else (0, 0, 255)
                cv2.rectangle(debug_img, (xmin, ymin), (xmax, ymax), color, 2)

                cv2.putText(
                    debug_img,
                    f"C{cluster.cluster_id}",
                    (xmin, ymax + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

        for assoc in associations:
            cluster = next(
                (c for c in lidar_clusters if c.cluster_id == assoc.lidar_cluster_id),
                None,
            )
            detection = next(
                (d for d in camera_detections if d.detection_id == assoc.camera_detection_id),
                None,
            )

            if cluster is not None and detection is not None:
                centroid_2d = projection_utils.project_point_3d_to_2d(
                    cluster.centroid, check_bounds=False
                )
                if centroid_2d is not None:
                    cx, cy = int(centroid_2d[0]), int(centroid_2d[1])
                    dx = int((detection.bbox_2d[0] + detection.bbox_2d[2]) / 2)
                    dy = int((detection.bbox_2d[1] + detection.bbox_2d[3]) / 2)
                    cv2.line(debug_img, (cx, cy), (dx, dy), (0, 255, 255), 2)

        legend_y = 30
        cv2.putText(
            debug_img,
            "Blue: Camera",
            (10, legend_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
        )
        cv2.putText(
            debug_img,
            "Green: Matched LiDAR",
            (10, legend_y + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            debug_img,
            "Red: Unmatched LiDAR",
            (10, legend_y + 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )
        cv2.putText(
            debug_img,
            f"Associations: {len(associations)}",
            (10, legend_y + 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        return debug_img
