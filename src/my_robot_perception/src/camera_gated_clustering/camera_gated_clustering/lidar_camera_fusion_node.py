#!/usr/bin/env python3
"""Fuse LiDAR 3D detections with camera imagery for classification."""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseWithCovariance
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
import cv_bridge
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from message_filters import ApproximateTimeSynchronizer, Subscriber

from camera_gated_clustering.projection import project_points


class LidarCameraFusionNode(Node):
    """Project LiDAR 3D detections into the camera image and attach labels."""

    def __init__(self) -> None:
        super().__init__('lidar_camera_fusion_node')

        self._declare_parameters()
        self._load_parameters()

        self.bridge = cv_bridge.CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.camera_matrix: Optional[np.ndarray] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._setup_publishers()
        self._setup_subscribers()

        self._last_runtime_warn = 0.0
        if self.runtime_check_enabled:
            self.create_timer(2.0, self._runtime_check)

        self.get_logger().info('LiDAR-camera fusion node ready.')

    def _declare_parameters(self) -> None:
        self.declare_parameter('frames.lidar_frame', 'os_sensor')
        self.declare_parameter('frames.camera_optical_frame', 'zed_left_camera_optical_frame')

        self.declare_parameter('topics.detections3d_in', '/lidar/detections3d')
        self.declare_parameter('topics.camera_image', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('topics.camera_info', '/zed/zed_node/left/camera_info')
        self.declare_parameter('topics.fused_detections3d', '/fusion/detections3d')
        self.declare_parameter('topics.debug_image', '/fusion/debug_image')
        self.declare_parameter('topics.visualization_markers', '/fusion/markers')

        self.declare_parameter('projection.min_depth_m', 0.5)
        self.declare_parameter('projection.max_depth_m', 25.0)
        self.declare_parameter('projection.image_margin_px', 20)

        self.declare_parameter('classifier.area_threshold_px', 4500)
        self.declare_parameter('classifier.min_roi_area_px', 900)
        self.declare_parameter('classifier.unknown_score', 0.2)
        self.declare_parameter('classifier.vehicle_score', 0.7)
        self.declare_parameter('classifier.person_score', 0.6)

        self.declare_parameter('gating_fusion.drop_if_no_projection', False)
        self.declare_parameter('gating_fusion.fallback_publish_unmodified', True)

        self.declare_parameter('sync.queue_size', 10)
        self.declare_parameter('sync.max_time_difference_ms', 100)

        self.declare_parameter('debug.publish_debug_image', True)

        self.declare_parameter('visualization.publish_markers', True)
        self.declare_parameter('visualization.marker_lifetime_sec', 0.2)
        self.declare_parameter('visualization.color_fused', [0.0, 0.8, 1.0])

        self.declare_parameter('runtime_check.enabled', True)
        self.declare_parameter('runtime_check.warn_period_sec', 5.0)

    def _load_parameters(self) -> None:
        self.frames = {
            'lidar': self.get_parameter('frames.lidar_frame').value,
            'camera_optical': self.get_parameter('frames.camera_optical_frame').value,
        }
        self.topics = {
            'detections': self.get_parameter('topics.detections3d_in').value,
            'image': self.get_parameter('topics.camera_image').value,
            'camera_info': self.get_parameter('topics.camera_info').value,
            'fused_detections': self.get_parameter('topics.fused_detections3d').value,
            'debug_image': self.get_parameter('topics.debug_image').value,
            'markers': self.get_parameter('topics.visualization_markers').value,
        }
        self.projection = {
            'min_depth': float(self.get_parameter('projection.min_depth_m').value),
            'max_depth': float(self.get_parameter('projection.max_depth_m').value),
            'margin_px': int(self.get_parameter('projection.image_margin_px').value),
        }
        self.classifier = {
            'area_threshold_px': int(self.get_parameter('classifier.area_threshold_px').value),
            'min_roi_area_px': int(self.get_parameter('classifier.min_roi_area_px').value),
            'unknown_score': float(self.get_parameter('classifier.unknown_score').value),
            'vehicle_score': float(self.get_parameter('classifier.vehicle_score').value),
            'person_score': float(self.get_parameter('classifier.person_score').value),
        }
        self.gating = {
            'drop_if_no_projection': bool(
                self.get_parameter('gating_fusion.drop_if_no_projection').value
            ),
            'fallback_publish_unmodified': bool(
                self.get_parameter('gating_fusion.fallback_publish_unmodified').value
            ),
        }
        self.sync = {
            'queue_size': int(self.get_parameter('sync.queue_size').value),
            'slop': float(self.get_parameter('sync.max_time_difference_ms').value) / 1000.0,
        }
        self.publish_debug_image = bool(self.get_parameter('debug.publish_debug_image').value)
        self.publish_markers = bool(self.get_parameter('visualization.publish_markers').value)
        self.marker_lifetime = float(self.get_parameter('visualization.marker_lifetime_sec').value)
        self.marker_color = self.get_parameter('visualization.color_fused').value
        self.runtime_check_enabled = bool(self.get_parameter('runtime_check.enabled').value)
        self.runtime_warn_period = float(self.get_parameter('runtime_check.warn_period_sec').value)

    def _setup_publishers(self) -> None:
        self.fused_pub = self.create_publisher(
            Detection3DArray,
            self.topics['fused_detections'],
            10,
        )
        self.debug_pub = self.create_publisher(Image, self.topics['debug_image'], 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.topics['markers'], 10)

    def _setup_subscribers(self) -> None:
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.topics['camera_info'],
            self.camera_info_callback,
            10,
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.sync['queue_size'],
        )

        self.detections_sub = Subscriber(
            self,
            Detection3DArray,
            self.topics['detections'],
            qos_profile=qos_profile,
        )
        self.image_sub = Subscriber(self, Image, self.topics['image'])

        self.syncer = ApproximateTimeSynchronizer(
            [self.detections_sub, self.image_sub],
            queue_size=self.sync['queue_size'],
            slop=self.sync['slop'],
        )
        self.syncer.registerCallback(self.fusion_callback)

    def _runtime_check(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_runtime_warn < self.runtime_warn_period:
            return

        if not self.tf_buffer.can_transform(
            self.frames['camera_optical'],
            self.frames['lidar'],
            Time(),
            timeout=Duration(seconds=0.1),
        ):
            self._last_runtime_warn = now
            self.get_logger().warning(
                f"TF missing: cannot transform from {self.frames['lidar']} "
                f"to {self.frames['camera_optical']}. Is the ZED driver running "
                "and the static calibration TF published?"
            )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)

    def fusion_callback(self, detections_msg: Detection3DArray, image_msg: Image) -> None:
        if self.camera_info is None or self.camera_matrix is None:
            if self.gating['fallback_publish_unmodified']:
                self.fused_pub.publish(detections_msg)
            self.get_logger().warning('CameraInfo not received yet, skipping fusion.')
            return

        lookup_time = detections_msg.header.stamp
        if lookup_time.sec == 0 and lookup_time.nanosec == 0:
            lookup_time = Time()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.frames['camera_optical'],
                detections_msg.header.frame_id or self.frames['lidar'],
                lookup_time,
                Duration(seconds=0.1),
            )
        except Exception as exc:  # pylint: disable=broad-except
            if self.gating['fallback_publish_unmodified']:
                self.fused_pub.publish(detections_msg)
            self.get_logger().warning(f'TF lookup failed: {exc}')
            return

        debug_image = None
        if self.publish_debug_image:
            try:
                debug_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            except cv_bridge.CvBridgeError as exc:
                self.get_logger().warning(f'Failed to convert debug image: {exc}')
                debug_image = None

        fused_array = Detection3DArray()
        fused_array.header = detections_msg.header

        marker_array = MarkerArray()
        marker_header = detections_msg.header
        if not marker_header.frame_id:
            marker_header.frame_id = self.frames['lidar']

        image_width = self.camera_info.width
        image_height = self.camera_info.height

        for detection in detections_msg.detections:
            projected_bbox = self._project_detection_to_image(
                detection,
                transform,
                detections_msg.header,
            )
            if projected_bbox is None:
                if not self.gating['drop_if_no_projection']:
                    fused_array.detections.append(detection)
                continue

            x_min, y_min, x_max, y_max = projected_bbox
            roi_area = max(0, x_max - x_min) * max(0, y_max - y_min)
            if roi_area < self.classifier['min_roi_area_px']:
                if not self.gating['drop_if_no_projection']:
                    fused_array.detections.append(detection)
                continue

            label, score = self._classify_roi(roi_area)
            fused_detection = self._decorate_detection(detection, label, score)
            fused_array.detections.append(fused_detection)

            if self.publish_markers:
                marker_array.markers.extend(
                    self._build_markers(fused_detection, marker_header, len(marker_array.markers))
                )

            if debug_image is not None:
                x_min_i = int(np.clip(x_min, 0, image_width - 1))
                y_min_i = int(np.clip(y_min, 0, image_height - 1))
                x_max_i = int(np.clip(x_max, 0, image_width - 1))
                y_max_i = int(np.clip(y_max, 0, image_height - 1))
                cv2.rectangle(debug_image, (x_min_i, y_min_i), (x_max_i, y_max_i), (0, 255, 255), 2)
                cv2.putText(
                    debug_image,
                    f'{label} {score:.2f}',
                    (x_min_i, max(0, y_min_i - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        self.fused_pub.publish(fused_array)

        if self.publish_markers:
            if not marker_array.markers:
                marker_array.markers.append(self._empty_marker(marker_header))
            self.marker_pub.publish(marker_array)

        if debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = image_msg.header
                self.debug_pub.publish(debug_msg)
            except cv_bridge.CvBridgeError as exc:
                self.get_logger().warning(f'Failed to publish debug image: {exc}')

    def _project_detection_to_image(
        self,
        detection: Detection3D,
        transform,
        fallback_header,
    ) -> Optional[tuple[float, float, float, float]]:
        corners = self._bbox_corners(detection)
        if corners.size == 0:
            return None

        header = detection.header if detection.header.frame_id else fallback_header
        points_cam = []
        for corner in corners:
            point_msg = PointStamped()
            point_msg.header = header
            point_msg.point.x = float(corner[0])
            point_msg.point.y = float(corner[1])
            point_msg.point.z = float(corner[2])
            transformed = do_transform_point(point_msg, transform)
            points_cam.append([
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            ])

        points_cam = np.array(points_cam, dtype=np.float32)
        depth_mask = (points_cam[:, 2] > self.projection['min_depth']) & (
            points_cam[:, 2] < self.projection['max_depth']
        )
        if not np.any(depth_mask):
            return None

        pixels, valid = project_points(points_cam, self.camera_matrix)
        valid = valid & depth_mask
        if not np.any(valid):
            return None

        pixels = pixels[valid]
        x_min = float(np.min(pixels[:, 0]))
        y_min = float(np.min(pixels[:, 1]))
        x_max = float(np.max(pixels[:, 0]))
        y_max = float(np.max(pixels[:, 1]))

        margin = self.projection['margin_px']
        x_min -= margin
        y_min -= margin
        x_max += margin
        y_max += margin

        return x_min, y_min, x_max, y_max

    def _bbox_corners(self, detection: Detection3D) -> np.ndarray:
        size = detection.bbox.size
        center = detection.bbox.center.position

        half_x = size.x * 0.5
        half_y = size.y * 0.5
        half_z = size.z * 0.5

        corners = np.array([
            [center.x - half_x, center.y - half_y, center.z - half_z],
            [center.x - half_x, center.y - half_y, center.z + half_z],
            [center.x - half_x, center.y + half_y, center.z - half_z],
            [center.x - half_x, center.y + half_y, center.z + half_z],
            [center.x + half_x, center.y - half_y, center.z - half_z],
            [center.x + half_x, center.y - half_y, center.z + half_z],
            [center.x + half_x, center.y + half_y, center.z - half_z],
            [center.x + half_x, center.y + half_y, center.z + half_z],
        ], dtype=np.float32)
        return corners

    def _classify_roi(self, roi_area: float) -> tuple[str, float]:
        if math.isnan(roi_area) or roi_area <= 0:
            return 'unknown', self.classifier['unknown_score']
        if roi_area >= self.classifier['area_threshold_px']:
            return 'vehicle', self.classifier['vehicle_score']
        return 'person', self.classifier['person_score']

    def _decorate_detection(self, detection: Detection3D, label: str, score: float) -> Detection3D:
        updated = Detection3D()
        updated.header = detection.header
        updated.bbox = detection.bbox
        updated.id = detection.id

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        hypothesis.hypothesis.score = float(score)

        pose = PoseWithCovariance()
        pose.pose = detection.bbox.center
        hypothesis.pose = pose

        updated.results = [hypothesis]
        return updated

    def _build_markers(
        self,
        detection: Detection3D,
        header,
        base_id: int,
    ) -> list[Marker]:
        color = self._marker_color()
        lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        marker_id = base_id

        cube = Marker()
        cube.header = header
        cube.ns = 'fusion_bbox'
        cube.id = marker_id
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose = detection.bbox.center
        cube.scale = detection.bbox.size
        cube.color = color
        cube.lifetime = lifetime

        label_marker = Marker()
        label_marker.header = header
        label_marker.ns = 'fusion_label'
        label_marker.id = marker_id + 1
        label_marker.type = Marker.TEXT_VIEW_FACING
        label_marker.action = Marker.ADD
        label_marker.pose = detection.bbox.center
        label_marker.pose.position.z += detection.bbox.size.z * 0.6
        label_marker.scale.z = max(0.2, detection.bbox.size.z * 0.2)
        label_marker.color = color
        label_marker.lifetime = lifetime
        label_marker.text = self._label_text(detection)

        return [cube, label_marker]

    def _marker_color(self):
        color = self.marker_color
        msg = Marker().color
        msg.r = float(color[0]) if len(color) > 0 else 0.0
        msg.g = float(color[1]) if len(color) > 1 else 0.8
        msg.b = float(color[2]) if len(color) > 2 else 1.0
        msg.a = 0.8
        return msg

    def _label_text(self, detection: Detection3D) -> str:
        if detection.results:
            hypothesis = detection.results[0].hypothesis
            return f"{hypothesis.class_id} {hypothesis.score:.2f}"
        return 'unknown'

    def _empty_marker(self, header) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = 'fusion_bbox'
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
