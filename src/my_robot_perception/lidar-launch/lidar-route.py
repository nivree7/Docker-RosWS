#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PointCloudRepublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_republisher')

        # BEST_EFFORT QoS to match most LiDAR drivers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.cloud_callback,
            qos
        )

        # Publisher
        self.pub = self.create_publisher(PointCloud2, '/input_pointcloud', qos)

        self.get_logger().info("Republisher initialized. Waiting for messages...")

    def cloud_callback(self, msg: PointCloud2):
        self.pub.publish(msg)

        # Print first 5 points for verification
        points = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
        first_points = [p for i,p in enumerate(points) if i<5]
        self.get_logger().info(f"First 5 points: {first_points}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
