#!/usr/bin/env python3
"""Camera perception node.

Subscribes to ZED left image + depth image, runs YOLOv8 inference, lifts
2D detections to 3D using the stereo depth map, and publishes a
Detection3DArray in the camera optical frame.
"""

from __future__ import annotations

import math
import os
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

import cv_bridge
from message_filters import ApproximateTimeSynchronizer, Subscriber


# ---------------------------------------------------------------------------
# Helper: locate model file via ament share directory
# ---------------------------------------------------------------------------
def _find_model(model_name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('camera_perception')
    path = os.path.join(share, 'models', model_name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'YOLO model not found at: {path}')
    return path


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class CameraPerceptionNode(Node):
    """YOLO + ZED stereo depth → 3D bounding boxes."""

    def __init__(self) -> None:
        super().__init__('camera_perception_node')

        self._declare_parameters()
        self._load_parameters()
        self._load_yolo_model()

        self.bridge = cv_bridge.CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.frame_counter = 0

        self._setup_publishers()
        self._setup_subscribers()

        self.get_logger().info(
            f'camera_perception_node ready. '
            f'Device={self._device}, frame_skip={self._frame_skip}'
        )

    # ------------------------------------------------------------------
    # Parameter declaration
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        # Frames
        self.declare_parameter('frames.camera_frame', 'zed_left_camera_optical_frame')

        # Topics
        self.declare_parameter('topics.camera_image', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('topics.depth_image', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('topics.camera_info', '/zed/zed_node/left/camera_info')
        self.declare_parameter('topics.detections3d_out', '/camera/detections3d')
        self.declare_parameter('topics.debug_image_out', '/camera/debug_image')
        self.declare_parameter('topics.markers_out', '/camera/detections_markers')

        # Detector
        self.declare_parameter('detector.enabled', True)
        self.declare_parameter('detector.model_name', 'yolov8_road_hazards.pt')
        self.declare_parameter('detector.confidence_threshold', 0.45)
        self.declare_parameter('detector.iou_threshold', 0.45)
        self.declare_parameter('detector.device', '0')
        self.declare_parameter('detector.img_size', 640)
        self.declare_parameter('detector.max_det', 50)
        self.declare_parameter('detector.frame_skip', 1)

        # Depth
        self.declare_parameter('depth.min_depth_m', 0.5)
        self.declare_parameter('depth.max_depth_m', 12.0)
        self.declare_parameter('depth.min_valid_fraction', 0.10)
        self.declare_parameter('depth.depth_roi_shrink', 0.8)

        # 3D size estimation
        self.declare_parameter('bbox3d.depth_estimate_method', 'std')
        self.declare_parameter('bbox3d.fixed_depth_extent_m', 0.5)

        # Output
        self.declare_parameter('output.publish_debug_image', True)
        self.declare_parameter('output.label_font_scale', 0.7)
        self.declare_parameter('output.label_thickness', 2)

    def _load_parameters(self) -> None:
        self._camera_frame = self.get_parameter('frames.camera_frame').value

        self._image_topic = self.get_parameter('topics.camera_image').value
        self._depth_topic = self.get_parameter('topics.depth_image').value
        self._camera_info_topic = self.get_parameter('topics.camera_info').value
        self._det3d_topic = self.get_parameter('topics.detections3d_out').value
        self._debug_image_topic = self.get_parameter('topics.debug_image_out').value
        self._markers_topic = self.get_parameter('topics.markers_out').value

        self._detector_enabled = self.get_parameter('detector.enabled').value
        self._model_name = self.get_parameter('detector.model_name').value
        self._conf_thresh = self.get_parameter('detector.confidence_threshold').value
        self._iou_thresh = self.get_parameter('detector.iou_threshold').value
        self._device = self.get_parameter('detector.device').value
        self._img_size = self.get_parameter('detector.img_size').value
        self._max_det = self.get_parameter('detector.max_det').value
        self._frame_skip = int(self.get_parameter('detector.frame_skip').value)

        self._min_depth = self.get_parameter('depth.min_depth_m').value
        self._max_depth = self.get_parameter('depth.max_depth_m').value
        self._min_valid_frac = self.get_parameter('depth.min_valid_fraction').value
        self._roi_shrink = self.get_parameter('depth.depth_roi_shrink').value

        self._depth_method = self.get_parameter('bbox3d.depth_estimate_method').value
        self._fixed_depth_ext = self.get_parameter('bbox3d.fixed_depth_extent_m').value

        self._publish_debug = self.get_parameter('output.publish_debug_image').value
        self._font_scale = self.get_parameter('output.label_font_scale').value
        self._font_thickness = int(self.get_parameter('output.label_thickness').value)

    # ------------------------------------------------------------------
    # YOLO model loading
    # ------------------------------------------------------------------
    def _load_yolo_model(self) -> None:
        if not self._detector_enabled:
            self.yolo = None
            self.get_logger().warn('Detector disabled via parameter.')
            return

        try:
            from ultralytics import YOLO
            model_path = _find_model(self._model_name)
            self.yolo = YOLO(model_path)
            # Warm-up: run once on blank image so first real frame is fast
            dummy = np.zeros((self._img_size, self._img_size, 3), dtype=np.uint8)
            self.yolo(dummy, device=self._device, imgsz=self._img_size, verbose=False)
            self.get_logger().info(
                f'YOLO loaded: {model_path}  '
                f'classes={list(self.yolo.names.values())}  '
                f'device={self._device}'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model: {exc}')
            self.yolo = None

    # ------------------------------------------------------------------
    # Publishers and subscribers
    # ------------------------------------------------------------------
    def _setup_publishers(self) -> None:
        self.det3d_pub = self.create_publisher(Detection3DArray, self._det3d_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self._debug_image_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

    def _setup_subscribers(self) -> None:
        # ZED publishes sensor data as BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # CameraInfo is typically RELIABLE
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            10,
        )

        self.image_sub = Subscriber(
            self, Image, self._image_topic, qos_profile=sensor_qos
        )
        self.depth_sub = Subscriber(
            self, Image, self._depth_topic, qos_profile=sensor_qos
        )

        # 50 ms slop — ZED image and depth share the same timestamp
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=5,
            slop=0.05,
        )
        self.sync.registerCallback(self._sync_callback)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self.camera_info is None:
            self.get_logger().info(
                f'Got CameraInfo: {msg.width}×{msg.height}  '
                f'fx={msg.k[0]:.1f}  fy={msg.k[4]:.1f}'
            )
        self.camera_info = msg

    def _sync_callback(self, image_msg: Image, depth_msg: Image) -> None:
        if self.camera_info is None:
            return
        if self.yolo is None:
            return

        self.frame_counter += 1
        if self.frame_counter % self._frame_skip != 0:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Image conversion failed: {exc}')
            return

        try:
            # depth_registered is float32 in metres
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().warning(f'Depth conversion failed: {exc}')
            return

        det3d_array, debug_img, markers = self._process(bgr, depth, image_msg.header)
        self.det3d_pub.publish(det3d_array)
        self.markers_pub.publish(markers)

        if self._publish_debug and debug_img is not None:
            out = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            out.header = image_msg.header
            self.debug_image_pub.publish(out)

    # ------------------------------------------------------------------
    # Core processing
    # ------------------------------------------------------------------
    def _process(
        self,
        bgr: np.ndarray,
        depth: np.ndarray,
        header,
    ):
        """Run YOLO inference then lift each 2D detection to 3D via ZED depth."""
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        img_h, img_w = depth.shape[:2]

        det3d_array = Detection3DArray()
        det3d_array.header = header
        det3d_array.header.frame_id = self._camera_frame

        markers = MarkerArray()
        debug_img = bgr.copy() if self._publish_debug else None

        # --- YOLO inference ---
        try:
            results = self.yolo(
                bgr,
                conf=self._conf_thresh,
                iou=self._iou_thresh,
                imgsz=self._img_size,
                max_det=self._max_det,
                device=self._device,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().warning(f'YOLO inference failed: {exc}', throttle_duration_sec=5.0)
            return det3d_array, debug_img, markers

        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return det3d_array, debug_img, markers

        boxes_xyxy = result.boxes.xyxy.cpu().numpy()     # (N, 4)
        confidences = result.boxes.conf.cpu().numpy()    # (N,)
        class_ids = result.boxes.cls.cpu().numpy().astype(int)  # (N,)

        for i, (xyxy, conf, cls_id) in enumerate(
            zip(boxes_xyxy, confidences, class_ids)
        ):
            x1, y1, x2, y2 = (
                max(0, int(xyxy[0])),
                max(0, int(xyxy[1])),
                min(img_w - 1, int(xyxy[2])),
                min(img_h - 1, int(xyxy[3])),
            )
            if x2 <= x1 or y2 <= y1:
                continue

            class_name = self.yolo.names.get(cls_id, str(cls_id))

            # --- Sample depth in shrunk ROI ---
            Z, valid = self._sample_depth(depth, x1, y1, x2, y2)
            if not valid:
                # No reliable depth — still draw box on debug image but skip 3D
                if debug_img is not None:
                    self._draw_2d_box(debug_img, x1, y1, x2, y2, class_name, conf,
                                      z_label='no_depth', color=(0, 128, 255))
                continue

            # --- Back-project to 3D ---
            u_c = (x1 + x2) / 2.0
            v_c = (y1 + y2) / 2.0
            X = (u_c - cx) * Z / fx
            Y = (v_c - cy) * Z / fy

            # 3D size estimation
            w3d = max(0.05, (x2 - x1) * Z / fx)
            h3d = max(0.05, (y2 - y1) * Z / fy)
            d3d = self._estimate_depth_extent(depth, x1, y1, x2, y2)

            # --- Build Detection3D ---
            det = Detection3D()
            det.header = det3d_array.header
            det.bbox.center.position.x = X
            det.bbox.center.position.y = Y
            det.bbox.center.position.z = Z
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = w3d
            det.bbox.size.y = h3d
            det.bbox.size.z = d3d

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_name
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            det3d_array.detections.append(det)

            # --- Debug overlay ---
            if debug_img is not None:
                self._draw_2d_box(debug_img, x1, y1, x2, y2, class_name, conf,
                                  z_label=f'Z={Z:.1f}m', color=(0, 220, 0))

            # --- RViz marker (sphere at 3D centroid) ---
            markers.markers.append(
                self._make_sphere_marker(i, X, Y, Z, class_name, det3d_array.header)
            )

        return det3d_array, debug_img, markers

    # ------------------------------------------------------------------
    # Depth helpers
    # ------------------------------------------------------------------
    def _sample_depth(
        self, depth: np.ndarray, x1: int, y1: int, x2: int, y2: int
    ):
        """Return (median_depth, is_valid) for the detection ROI.

        Shrinks the pixel ROI inward to avoid boundary depth artefacts, then
        takes the median of valid (finite, in-range) pixels.
        """
        shrink = self._roi_shrink
        dx = int((x2 - x1) * (1.0 - shrink) / 2)
        dy = int((y2 - y1) * (1.0 - shrink) / 2)
        rx1, ry1 = x1 + dx, y1 + dy
        rx2, ry2 = x2 - dx, y2 - dy
        if rx2 <= rx1 or ry2 <= ry1:
            rx1, ry1, rx2, ry2 = x1, y1, x2, y2

        roi = depth[ry1:ry2, rx1:rx2]
        total = (ry2 - ry1) * (rx2 - rx1)
        valid = roi[np.isfinite(roi) & (roi >= self._min_depth) & (roi <= self._max_depth)]

        if len(valid) < max(1, int(total * self._min_valid_frac)):
            return 0.0, False

        return float(np.median(valid)), True

    def _estimate_depth_extent(
        self, depth: np.ndarray, x1: int, y1: int, x2: int, y2: int
    ) -> float:
        """Estimate the along-Z extent of the detection from depth spread."""
        if self._depth_method == 'fixed':
            return self._fixed_depth_ext

        roi = depth[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi) & (roi >= self._min_depth) & (roi <= self._max_depth)]
        if len(valid) < 4:
            return self._fixed_depth_ext

        # 2×std gives a rough front-to-back depth span; clamp to sensible range
        extent = float(np.std(valid)) * 2.0 + 0.05
        return float(np.clip(extent, 0.05, 2.0))

    # ------------------------------------------------------------------
    # Visualisation helpers
    # ------------------------------------------------------------------
    def _draw_2d_box(
        self,
        img: np.ndarray,
        x1: int, y1: int, x2: int, y2: int,
        class_name: str,
        conf: float,
        z_label: str = '',
        color=(0, 220, 0),
    ) -> None:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f'{class_name} {conf:.2f}'
        if z_label:
            label += f' {z_label}'
        cv2.putText(
            img, label, (x1, max(y1 - 6, 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            self._font_scale, color, self._font_thickness,
        )

    def _make_sphere_marker(
        self, idx: int, X: float, Y: float, Z: float,
        class_name: str, header
    ) -> Marker:
        m = Marker()
        m.header = header
        m.ns = 'camera_detections'
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = X
        m.pose.position.y = Y
        m.pose.position.z = Z
        m.pose.orientation.w = 1.0
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        m.color.a = 0.8
        # Colour by class index
        colours = [(0.0, 1.0, 0.0), (1.0, 1.0, 0.0), (0.0, 0.5, 1.0)]
        cls_idx = list(self.yolo.names.values()).index(class_name) if class_name in self.yolo.names.values() else 0
        r, g, b = colours[cls_idx % len(colours)]
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.lifetime.sec = 1
        return m


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
