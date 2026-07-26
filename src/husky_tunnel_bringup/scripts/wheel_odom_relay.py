#!/usr/bin/env python3

import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class WheelOdomRelay(Node):

    def __init__(self):
        super().__init__("wheel_odom_relay")

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(
            Odometry,
            "/lio_sam/odometry/imu_incremental",
            qos,
        )

        self.subscription = self.create_subscription(
            Odometry,
            "/a200_0000/platform/odom",
            self.odom_callback,
            qos,
        )

        self.received_first_message = False
        self.get_logger().info("Wheel odometry relay started")

    def odom_callback(self, message):
        self.publisher.publish(message)

        if not self.received_first_message:
            self.get_logger().info("Relaying Clearpath odometry to LIO-SAM")
            self.received_first_message = True


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

