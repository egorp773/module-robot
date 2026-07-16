"""Publish only zero TwistStamped commands for an explicitly bounded period."""

import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ZeroCommand(Node):
    def __init__(self) -> None:
        super().__init__("zero_command")
        self.declare_parameter("topic", "/cmd_vel_manual")
        self.declare_parameter("duration_s", 1.0)
        self.declare_parameter("rate_hz", 20.0)
        topic = str(self.get_parameter("topic").value)
        self.done = False
        self._deadline = time.monotonic() + max(
            0.1, float(self.get_parameter("duration_s").value)
        )
        self._publisher = self.create_publisher(TwistStamped, topic, 10)
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"Publishing zero-only commands on {topic}")

    def _tick(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self._publisher.publish(msg)
        if time.monotonic() >= self._deadline:
            self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZeroCommand()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
