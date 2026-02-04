#!/usr/bin/env python3
"""
Main camera-gated clustering node.
Fuses LiDAR Euclidean clustering with camera object detection.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3, Quaternion
from fusion_msgs.msg import EnrichedCluster, EnrichedClusterArray, AssociationDebug
from visualization_msgs.msg import MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import cv_bridge
import time
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Import our modules
from camera_gated_clustering.projection_utils import ProjectionUtils
from camera_gated_clustering.cluster_association import (
    ClusterAssociator, LiDARCluster, CameraDetection
)
from camera_gated_clustering.gating_strategies import GatingFactory
from camera_gated_clustering.visualization import FusionVisualizer


class CameraGatedClusteringNode(Node):
    """Main node for camera-gated LiDAR clustering."""

    def __init__(self):
        super().__init__('camera_gated_clustering_node')

        print('[fusion-init] Node init start')

        # Declare parameters
        self._declare_parameters()
        print('[fusion-init] Parameters declared')

        # Load configuration
        self.config = self._load_config()
        print('[fusion-init] Config loaded')

        self.get_logger().info('Camera Gated Clustering Node Starting...')
        self.get_logger().info(f'Gating mode: {self.config["gating"]["mode"]}')
        self.get_logger().info(f'Association method: {self.config["association"]["method"]}')

        # Initialize utilities
        self.projection_utils = ProjectionUtils()
        self.associator = ClusterAssociator(self.config['association'])
        self.gating_strategy = GatingFactory.create(
            self.config['gating']['mode'],
            self.config['confidence']
        )
        self.visualizer = FusionVisualizer(self.config['visualization'])
        self.bridge = cv_bridge.CvBridge()

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.camera_info = None
        self.cluster_id_counter = 0
        self.sync_counter = 0
        self.last_sync_log = 0.0
        self.external_clusters_msg = None

        # Setup subscribers and publishers
        print('[fusion-init] Setting up subscribers/publishers')
        self._setup_subscribers()
        self._setup_publishers()
        print('[fusion-init] Subscribers/publishers ready')

        self.get_logger().info('Camera Gated Clustering Node Ready!')

    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        # Gating
        self.declare_parameter('gating.mode', 'soft')
        self.declare_parameter('gating.enabled', True)

        # Association
        self.declare_parameter('association.method', 'hybrid')
        self.declare_parameter('association.iou_threshold', 0.25)
        self.declare_parameter('association.max_centroid_distance_px', 100)
        self.declare_parameter('association.max_spatial_distance_m', 2.5)

        # Confidence
        self.declare_parameter('confidence.min_camera_confidence', 0.45)
        self.declare_parameter('confidence.min_lidar_confidence', 0.35)
        self.declare_parameter('confidence.camera_weight', 0.6)
        self.declare_parameter('confidence.lidar_weight', 0.4)

        # Clustering
        self.declare_parameter('clustering.enabled', True)
        self.declare_parameter('clustering.cluster_tolerance_m', 0.45)
        self.declare_parameter('clustering.min_cluster_size', 40)
        self.declare_parameter('clustering.max_cluster_size', 20000)
        self.declare_parameter('clustering.use_external', False)
        self.declare_parameter('clustering.external_topic', '/fusion/lidar_clusters')

        # Frames
        self.declare_parameter('frames.lidar_frame', 'os_sensor')
        self.declare_parameter('frames.camera_frame', 'zed_camera_center')
        self.declare_parameter('frames.base_frame', 'base_link')

        # Topics
        self.declare_parameter('topics.lidar_points', '/ouster/points')
        self.declare_parameter('topics.camera_image', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('topics.camera_info', '/zed/zed_node/left/camera_info')
        self.declare_parameter('topics.camera_objects', '/zed/zed_node/obj_det/objects')

        # Sync
        self.declare_parameter('sync.queue_size', 10)
        self.declare_parameter('sync.max_time_difference_ms', 100)

        # Visualization
        self.declare_parameter('visualization.publish_debug_image', True)
        self.declare_parameter('visualization.publish_markers', True)

    def _load_config(self) -> dict:
        """Load configuration from parameters."""
        config = {
            'gating': {
                'mode': self.get_parameter('gating.mode').value,
                'enabled': self.get_parameter('gating.enabled').value,
            },
            'association': {
                'method': self.get_parameter('association.method').value,
                'iou_threshold': self.get_parameter('association.iou_threshold').value,
                'max_centroid_distance_px': self.get_parameter('association.max_centroid_distance_px').value,
                'max_spatial_distance_m': self.get_parameter('association.max_spatial_distance_m').value,
                'weight_iou': 0.4,
                'weight_centroid': 0.3,
                'weight_spatial': 0.3,
                'min_hybrid_score': 0.5,
            },
            'confidence': {
                'min_camera_confidence': self.get_parameter('confidence.min_camera_confidence').value,
                'min_lidar_confidence': self.get_parameter('confidence.min_lidar_confidence').value,
                'camera_weight': self.get_parameter('confidence.camera_weight').value,
                'lidar_weight': self.get_parameter('confidence.lidar_weight').value,
                'min_cluster_points': self.get_parameter('clustering.min_cluster_size').value,
                'max_cluster_points': self.get_parameter('clustering.max_cluster_size').value,
            },
            'clustering': {
                'enabled': self.get_parameter('clustering.enabled').value,
                'cluster_tolerance_m': self.get_parameter('clustering.cluster_tolerance_m').value,
                'min_cluster_size': self.get_parameter('clustering.min_cluster_size').value,
                'max_cluster_size': self.get_parameter('clustering.max_cluster_size').value,
                'use_external': self.get_parameter('clustering.use_external').value,
                'external_topic': self.get_parameter('clustering.external_topic').value,
            },
            'frames': {
                'lidar_frame': self.get_parameter('frames.lidar_frame').value,
                'camera_frame': self.get_parameter('frames.camera_frame').value,
                'base_frame': self.get_parameter('frames.base_frame').value,
            },
            'topics': {
                'lidar_points': self.get_parameter('topics.lidar_points').value,
                'camera_image': self.get_parameter('topics.camera_image').value,
                'camera_info': self.get_parameter('topics.camera_info').value,
                'camera_objects': self.get_parameter('topics.camera_objects').value,
            },
            'sync': {
                'queue_size': self.get_parameter('sync.queue_size').value,
                'max_time_difference_ms': self.get_parameter('sync.max_time_difference_ms').value,
            },
            'visualization': {
                'publish_debug_image': self.get_parameter('visualization.publish_debug_image').value,
                'publish_markers': self.get_parameter('visualization.publish_markers').value,
                'marker_lifetime_ms': 200,
                'color_lidar_only': [0.0, 1.0, 0.5],
                'color_camera_validated': [1.0, 0.8, 0.0],
                'color_camera_only': [0.0, 0.5, 1.0],
            },
        }
        return config

    def _setup_subscribers(self):
        """Setup synchronized subscribers for sensor data."""
        # Camera info (separate, no sync needed)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.config['topics']['camera_info'],
            self.camera_info_callback,
            10
        )

        # External clustering (optional)
        if self.config['clustering'].get('use_external', False):
            self.external_clusters_sub = self.create_subscription(
                EnrichedClusterArray,
                self.config['clustering'].get('external_topic', '/fusion/enriched_clusters'),
                self.external_clusters_callback,
                10
            )

        # Synchronized subscribers for fusion
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.config['sync']['queue_size']
        )

        self.lidar_sub = Subscriber(
            self,
            PointCloud2,
            self.config['topics']['lidar_points'],
            qos_profile=qos_profile
        )

        # Camera QoS - ZED publishes with RELIABLE, so we need to match
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.config['sync']['queue_size']
        )

        self.image_sub = Subscriber(
            self,
            Image,
            self.config['topics']['camera_image'],
            qos_profile=camera_qos
        )

        # Note: Replace with actual ZED objects message type
        # For now using Image as placeholder - you'll need to import zed_msgs
        # from zed_msgs.msg import ObjectsStamped
        # For demonstration, we'll use a simple approach

        # Approximate time synchronizer
        slop = self.config['sync']['max_time_difference_ms'] / 1000.0

        self.sync = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.image_sub],
            queue_size=self.config['sync']['queue_size'],
            slop=slop
        )

        self.sync.registerCallback(self.sensor_fusion_callback)

        self.get_logger().info('Subscribers configured with approximate time sync')

    def _setup_publishers(self):
        """Setup publishers for fusion results."""
        self.enriched_clusters_pub = self.create_publisher(
            EnrichedClusterArray,
            '/fusion/enriched_clusters',
            10
        )

        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2,
            '/fusion/filtered_cloud',
            10
        )

        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/fusion/markers',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/fusion/debug_image',
            10
        )

        self.get_logger().info('Publishers configured')

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics."""
        if self.camera_info is None:
            self.get_logger().info('Received camera info')
        self.camera_info = msg
        self.projection_utils.update_camera_info(msg)

    def external_clusters_callback(self, msg: EnrichedClusterArray):
        """Store latest external clustering message."""
        self.external_clusters_msg = msg

    def sensor_fusion_callback(self, lidar_msg: PointCloud2, image_msg: Image):
        """
        Main fusion callback - processes synchronized sensor data.

        Args:
            lidar_msg: LiDAR point cloud
            image_msg: Camera image
        """
        try:
            self.sync_counter += 1
            now = time.time()
            if now - self.last_sync_log > 5.0:
                self.get_logger().info(
                    'Sync callback firing. Count=%d, lidar_stamp=%s, image_stamp=%s',
                    self.sync_counter,
                    str(lidar_msg.header.stamp),
                    str(image_msg.header.stamp)
                )
                self.last_sync_log = now

            self.get_logger().debug('Processing sensor fusion...')

            # Check if camera info is available
            if self.camera_info is None:
                self.get_logger().warn('Waiting for camera info...', throttle_duration_sec=2.0)
                return

            # 1. Process LiDAR clustering
            if self.config['clustering'].get('use_external', False):
                if self.external_clusters_msg is None:
                    self.get_logger().warn('Waiting for external clusters...', throttle_duration_sec=2.0)
                    return
                lidar_clusters = self._convert_external_clusters(
                    self.external_clusters_msg,
                    lidar_msg.header
                )
            else:
                lidar_clusters = self.process_lidar_clusters(lidar_msg)

            if not lidar_clusters:
                self.get_logger().debug('No LiDAR clusters detected')
                return

            # 2. Get camera detections (placeholder - implement based on ZED API)
            camera_detections = self.get_camera_detections()

            # 3. Associate clusters with detections
            associations, unmatched_lidar, unmatched_camera = self.associator.associate(
                lidar_clusters,
                camera_detections,
                self.projection_utils
            )

            self.get_logger().debug(
                f'Associations: {len(associations)}, '
                f'Unmatched LiDAR: {len(unmatched_lidar)}, '
                f'Unmatched Camera: {len(unmatched_camera)}'
            )

            # 4. Create enriched clusters
            enriched_clusters = self.create_enriched_clusters(
                lidar_clusters,
                associations,
                lidar_msg.header
            )

            # 5. Apply gating
            if self.config['gating']['enabled']:
                enriched_clusters = self.gating_strategy.apply(
                    enriched_clusters,
                    associations,
                    unmatched_lidar
                )

            # 6. Publish results
            self.publish_enriched_clusters(enriched_clusters, lidar_msg.header)

            # 7. Publish visualization
            if self.config['visualization']['publish_markers']:
                markers = self.visualizer.create_marker_array(enriched_clusters, lidar_msg.header)
                self.markers_pub.publish(markers)

            # 8. Publish debug image
            if self.config['visualization']['publish_debug_image']:
                self.publish_debug_image(
                    image_msg,
                    lidar_clusters,
                    camera_detections,
                    associations
                )

            self.get_logger().debug(f'Published {len(enriched_clusters)} enriched clusters')

        except Exception as e:
            self.get_logger().error(f'Error in sensor fusion: {str(e)}')

    def process_lidar_clusters(self, lidar_msg: PointCloud2) -> list:
        """
        Process LiDAR point cloud and extract clusters.

        Args:
            lidar_msg: PointCloud2 message

        Returns:
            List of LiDARCluster objects
        """
        try:
            # Get transform from LiDAR to camera frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config['frames']['camera_frame'],
                    lidar_msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed: {str(e)}', throttle_duration_sec=2.0)
                return []

            # Convert ROS PointCloud2 to numpy array
            points_list = list(pc2.read_points(
                lidar_msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            ))

            if not points_list:
                self.get_logger().debug('LiDAR point cloud empty after filtering')
                return []

            points = np.array(points_list, dtype=np.float32)

            # Transform points to camera frame
            points_camera = self.projection_utils.transform_points_to_camera_frame(
                points,
                transform
            )

            # Simple clustering (placeholder - you can reuse existing euclidean clustering)
            clusters = self.simple_clustering(points_camera)

            return clusters

        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR: {str(e)}')
            return []

    def _convert_external_clusters(self, msg: EnrichedClusterArray, header) -> list:
        """Convert external clustering output to LiDARCluster list."""
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config['frames']['camera_frame'],
                    header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'TF lookup failed: {str(e)}', throttle_duration_sec=2.0)
                return []

            if not msg.clusters:
                return []

            centroids = np.array(
                [[c.centroid.x, c.centroid.y, c.centroid.z] for c in msg.clusters],
                dtype=np.float32
            )
            centroids_cam = self.projection_utils.transform_points_to_camera_frame(
                centroids,
                transform
            )

            lidar_clusters = []
            for idx, cluster_msg in enumerate(msg.clusters):
                centroid_cam = centroids_cam[idx]
                dimensions = np.array(
                    [
                        cluster_msg.dimensions.x,
                        cluster_msg.dimensions.y,
                        cluster_msg.dimensions.z,
                    ],
                    dtype=np.float32
                )

                confidence = cluster_msg.lidar_confidence
                lidar_clusters.append(
                    LiDARCluster(
                        cluster_id=cluster_msg.cluster_id,
                        centroid=centroid_cam,
                        dimensions=dimensions,
                        bbox_2d=None,
                        point_count=cluster_msg.point_count,
                        confidence=confidence,
                    )
                )

            return lidar_clusters

        except Exception as e:
            self.get_logger().error(f'Error converting external clusters: {str(e)}')
            return []

    def simple_clustering(self, points: np.ndarray) -> list:
        """
        Simple distance-based clustering (placeholder).
        Replace with your existing Euclidean clustering logic.

        Args:
            points: Nx3 numpy array of points

        Returns:
            List of LiDARCluster objects
        """
        clusters = []

        # TODO: Implement proper clustering or integrate with existing clustering node
        # For now, create a simple cluster from all points

        if len(points) < self.config['clustering']['min_cluster_size']:
            return clusters

        # Calculate cluster properties
        centroid = np.mean(points, axis=0)
        min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        dimensions = max_coords - min_coords

        # Calculate confidence based on point count and distance
        distance = np.linalg.norm(centroid)
        confidence = self.gating_strategy.calculate_cluster_confidence(
            point_count=len(points),
            average_distance=distance
        )

        # Project to 2D
        bbox_2d = self.projection_utils.project_3d_bbox_to_2d(centroid, dimensions)

        cluster = LiDARCluster(
            cluster_id=self.cluster_id_counter,
            centroid=centroid,
            dimensions=dimensions,
            bbox_2d=bbox_2d,
            point_count=len(points),
            confidence=confidence
        )

        self.cluster_id_counter += 1
        clusters.append(cluster)

        return clusters

    def get_camera_detections(self) -> list:
        """
        Get camera object detections.
        This is a placeholder - implement based on ZED ObjectsStamped message.

        Returns:
            List of CameraDetection objects
        """
        # TODO: Subscribe to ZED object detection topic
        # Parse ObjectsStamped message
        # For now, return empty list
        return []

    def create_enriched_clusters(
        self,
        lidar_clusters: list,
        associations: list,
        header: Header
    ) -> list:
        """
        Create enriched cluster messages.

        Args:
            lidar_clusters: List of LiDARCluster objects
            associations: List of Association objects
            header: Message header

        Returns:
            List of enriched cluster objects (with attributes set for publishing)
        """
        # Create association map
        assoc_map = {a.lidar_cluster_id: a for a in associations}

        enriched_clusters = []

        for cluster in lidar_clusters:
            # Create enriched cluster object
            enriched = type('EnrichedCluster', (), {})()

            enriched.cluster_id = cluster.cluster_id
            enriched.centroid = Point(
                x=float(cluster.centroid[0]),
                y=float(cluster.centroid[1]),
                z=float(cluster.centroid[2])
            )
            enriched.dimensions = Vector3(
                x=float(cluster.dimensions[0]),
                y=float(cluster.dimensions[1]),
                z=float(cluster.dimensions[2])
            )
            enriched.point_count = cluster.point_count
            enriched.lidar_confidence = cluster.confidence
            enriched.average_distance = float(np.linalg.norm(cluster.centroid))

            # Check if associated with camera
            if cluster.cluster_id in assoc_map:
                assoc = assoc_map[cluster.cluster_id]
                enriched.camera_validated = True
                enriched.object_class = assoc.object_class
                enriched.camera_confidence = assoc.camera_confidence
                enriched.association_score = assoc.combined_score

                # Set 2D bbox if available
                if cluster.bbox_2d is not None:
                    enriched.bbox_2d_xmin = cluster.bbox_2d[0]
                    enriched.bbox_2d_ymin = cluster.bbox_2d[1]
                    enriched.bbox_2d_xmax = cluster.bbox_2d[2]
                    enriched.bbox_2d_ymax = cluster.bbox_2d[3]
            else:
                enriched.camera_validated = False
                enriched.object_class = "unknown"
                enriched.camera_confidence = 0.0
                enriched.association_score = 0.0
                enriched.bbox_2d_xmin = 0.0
                enriched.bbox_2d_ymin = 0.0
                enriched.bbox_2d_xmax = 0.0
                enriched.bbox_2d_ymax = 0.0

            # Calculate fusion confidence (will be set by gating strategy)
            enriched.fusion_confidence = 0.0

            enriched_clusters.append(enriched)

        return enriched_clusters

    def publish_enriched_clusters(self, enriched_clusters: list, header: Header):
        """Publish enriched cluster array."""
        msg = EnrichedClusterArray()
        msg.header = header

        for enriched in enriched_clusters:
            cluster_msg = EnrichedCluster()
            cluster_msg.header = header
            cluster_msg.centroid = enriched.centroid
            cluster_msg.dimensions = enriched.dimensions
            cluster_msg.orientation = Quaternion(w=1.0)
            cluster_msg.object_class = enriched.object_class
            cluster_msg.camera_confidence = enriched.camera_confidence
            cluster_msg.camera_validated = enriched.camera_validated
            cluster_msg.point_count = enriched.point_count
            cluster_msg.lidar_confidence = enriched.lidar_confidence
            cluster_msg.average_distance = enriched.average_distance
            cluster_msg.fusion_confidence = enriched.fusion_confidence
            cluster_msg.cluster_id = enriched.cluster_id
            cluster_msg.association_score = enriched.association_score
            cluster_msg.bbox_2d_xmin = enriched.bbox_2d_xmin
            cluster_msg.bbox_2d_ymin = enriched.bbox_2d_ymin
            cluster_msg.bbox_2d_xmax = enriched.bbox_2d_xmax
            cluster_msg.bbox_2d_ymax = enriched.bbox_2d_ymax

            msg.clusters.append(cluster_msg)

        msg.total_clusters = len(enriched_clusters)
        msg.camera_validated_count = sum(1 for c in enriched_clusters if c.camera_validated)
        msg.lidar_only_count = msg.total_clusters - msg.camera_validated_count
        msg.average_fusion_confidence = (
            sum(c.fusion_confidence for c in enriched_clusters) / len(enriched_clusters)
            if enriched_clusters else 0.0
        )

        self.enriched_clusters_pub.publish(msg)

    def publish_debug_image(
        self,
        image_msg: Image,
        lidar_clusters: list,
        camera_detections: list,
        associations: list
    ):
        """Publish debug image with overlays."""
        try:
            # Convert ROS image to CV image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # Create debug visualization
            debug_img = self.visualizer.create_debug_image(
                cv_image,
                lidar_clusters,
                camera_detections,
                associations,
                self.projection_utils
            )

            # Convert back to ROS message
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = image_msg.header

            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error creating debug image: {str(e)}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = CameraGatedClusteringNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
