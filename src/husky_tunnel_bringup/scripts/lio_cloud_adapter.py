#!/usr/bin/env python3

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class LioCloudAdapter(Node):

    def __init__(self):
        super().__init__("lio_cloud_adapter")

        self.publisher = self.create_publisher(
            PointCloud2,
            "/lio_sam/points",
            qos_profile_sensor_data,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            "/a200_0000/sensors/lidar3d_0/points",
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.reported_layout = False
        self.last_stamp_ns = None
        self.scan_period = 0.1
        self.get_logger().info("LIO-SAM cloud adapter started")

    def cloud_callback(self, msg):
        if msg.point_step != 32:
            self.get_logger().error(
                f"Expected point_step 32, received {msg.point_step}"
            )
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        expected_size = msg.height * msg.row_step
        if raw.size < expected_size:
            self.get_logger().error("Point cloud data is smaller than expected")
            return

        rows = raw[:expected_size].reshape(msg.height, msg.row_step)
        records = rows[:, :msg.width * msg.point_step]
        records = records.reshape(-1, msg.point_step)

        float_type = np.dtype(">f4" if msg.is_bigendian else "<f4")
        xyz = records[:, :12].copy().view(float_type).reshape(-1, 3)

        valid_mask = np.isfinite(xyz).all(axis=1)
        filtered = records[valid_mask].copy()

        # Estimate the scan period from consecutive simulation timestamps.
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )

        if self.last_stamp_ns is not None:
            measured_period = (
                stamp_ns - self.last_stamp_ns
            ) * 1.0e-9

            if 0.02 <= measured_period <= 0.20:
                self.scan_period = measured_period

        self.last_stamp_ns = stamp_ns

        # The Gazebo cloud is organized as:
        #   height = laser rings
        #   width  = horizontal firing columns
        #
        # Assign each column a relative firing time across the scan.
        columns = np.tile(
            np.arange(msg.width, dtype=np.float32),
            msg.height,
        )

        relative_time = (
            columns / float(msg.width)
        ) * self.scan_period

        filtered_time = relative_time[valid_mask]
        time_bytes = np.asarray(
            filtered_time,
            dtype=float_type,
        ).view(np.uint8).reshape(-1, 4)

        # Bytes 28–31 are padding in the Gazebo cloud.
        filtered[:, 28:32] = time_bytes

        output = PointCloud2()
        output.header = msg.header
        output.height = 1
        output.width = int(filtered.shape[0])
        output.fields = list(msg.fields)
        output.fields.append(
            PointField(
                name="time",
                offset=28,
                datatype=PointField.FLOAT32,
                count=1,
            )
        )
        output.is_bigendian = msg.is_bigendian
        output.point_step = 32
        output.row_step = output.width * output.point_step
        output.data = filtered.tobytes()
        output.is_dense = True

        self.publisher.publish(output)

        if not self.reported_layout:
            removed = records.shape[0] - filtered.shape[0]
            self.get_logger().info(
                f"Input points: {records.shape[0]}, "
                f"valid points: {filtered.shape[0]}, "
                f"removed: {removed}, "
                f"scan period: {self.scan_period:.6f} s, "
                f"point time: {filtered_time.min():.6f} to "
                f"{filtered_time.max():.6f} s"
            )
            self.reported_layout = True


def main(args=None):
    rclpy.init(args=args)
    node = LioCloudAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
