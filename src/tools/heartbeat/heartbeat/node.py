import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Heartbeat(Node):
    def __init__(self):
        super().__init__("heartbeat")

        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("topic", "/system/heartbeat")

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._topic = str(self.get_parameter("topic").value)

        self._pub = self.create_publisher(String, self._topic, 10)
        period_s = 1.0 / max(rate_hz, 0.1)

        self._timer = self.create_timer(period_s, self._tick)
        self.get_logger().info(f"Publishing heartbeat on {self._topic} at {rate_hz} Hz")

    def _tick(self):
        msg = String()
        msg.data = "alive"
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = Heartbeat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
