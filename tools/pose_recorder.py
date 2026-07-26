#!/usr/bin/env python3
"""
pose_recorder.py

Records three trajectories during a live exploration run, each to its own
CSV, timestamped in seconds (ROS/sim time):

  1. ground_truth  -- the robot's TRUE pose in Gazebo. This is something
     you'd never have on real hardware; simulation gives it to us for
     free, via Gazebo's own transport topic (bridged into ROS first --
     see the bridge command below).
  2. lio_sam       -- LIO-SAM's estimated odometry (nav_msgs/Odometry).
  3. slam_toolbox  -- SLAM Toolbox's estimated pose, sampled via a TF
     lookup (map -> base_link) on a timer, since slam_toolbox doesn't
     publish a dedicated pose topic by default.

Run `ate_rpe.py` afterwards on the resulting CSVs to compute ATE/RPE.

IMPORTANT -- topic names to verify before running:
  Ground truth requires a one-time bridge (Gazebo doesn't publish this to
  ROS by default). In a separate terminal, with the exploration launch
  already running:

      source /opt/ros/humble/setup.bash
      source ~/husky_ws/install/setup.bash
      ros2 run ros_gz_bridge parameter_bridge \\
        "/world/tunnel/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"

  (Replace "tunnel" with your actual world name if different -- check
  with `ign topic -l | grep dynamic_pose`.)

  Also verify the LIO-SAM odometry topic name for your build:
      ros2 topic list | grep -i lio_sam
  Default assumed here is /lio_sam/mapping/odometry -- override with
  --ros-args -p lio_sam_odom_topic:=/your/actual/topic if different.

Usage:
    python3 pose_recorder.py --ros-args \\
      -p use_sim_time:=true \\
      -p output_dir:=/home/you/husky_ws/results/pose_logs
"""

import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class PoseRecorder(Node):

    def __init__(self):
        super().__init__("pose_recorder")

        self.declare_parameter("output_dir", os.path.expanduser("~/husky_ws/results/pose_logs"))
        self.declare_parameter("robot_model_name", "a200_0000")
        self.declare_parameter("ground_truth_topic", "/world/tunnel/dynamic_pose/info")
        self.declare_parameter("lio_sam_odom_topic", "/lio_sam/mapping/odometry")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("slam_sample_period_s", 0.1)

        self.output_dir = self.get_parameter("output_dir").value
        self.robot_model_name = self.get_parameter("robot_model_name").value
        self.ground_truth_topic = self.get_parameter("ground_truth_topic").value
        self.lio_sam_odom_topic = self.get_parameter("lio_sam_odom_topic").value
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.slam_sample_period_s = self.get_parameter("slam_sample_period_s").value

        os.makedirs(self.output_dir, exist_ok=True)

        self.gt_file = open(os.path.join(self.output_dir, "ground_truth.csv"), "w", newline="")
        self.gt_writer = csv.writer(self.gt_file)
        self.gt_writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])

        self.lio_file = open(os.path.join(self.output_dir, "lio_sam.csv"), "w", newline="")
        self.lio_writer = csv.writer(self.lio_file)
        self.lio_writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])

        self.slam_file = open(os.path.join(self.output_dir, "slam_toolbox.csv"), "w", newline="")
        self.slam_writer = csv.writer(self.slam_file)
        self.slam_writer.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])

        gt_qos = QoSProfile(depth=50)
        gt_qos.reliability = QoSReliabilityPolicy.RELIABLE

        # LIO-SAM's odometry publisher uses BEST_EFFORT reliability (common
        # for high-rate odometry/sensor topics) -- a default RELIABLE
        # subscriber is incompatible with it and silently receives nothing,
        # which is exactly what happened during testing. Match it here.
        lio_qos = QoSProfile(depth=50)
        lio_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.gt_count = 0
        self.lio_count = 0
        self.slam_count = 0

        self.gt_sub = self.create_subscription(
            TFMessage, self.ground_truth_topic, self.ground_truth_callback, gt_qos)
        self.lio_sub = self.create_subscription(
            Odometry, self.lio_sam_odom_topic, self.lio_sam_callback, lio_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.slam_timer = self.create_timer(
            self.slam_sample_period_s, self.slam_sample_callback)

        self.status_timer = self.create_timer(5.0, self.status_callback)

        self.get_logger().info(
            f"Recording to {self.output_dir}\n"
            f"  ground_truth_topic={self.ground_truth_topic} "
            f"(filtering for model '{self.robot_model_name}')\n"
            f"  lio_sam_odom_topic={self.lio_sam_odom_topic}\n"
            f"  slam via TF lookup {self.global_frame} -> {self.robot_frame} "
            f"every {self.slam_sample_period_s}s")

    def status_callback(self):
        self.get_logger().info(
            f"Recorded so far -- ground_truth: {self.gt_count}, "
            f"lio_sam: {self.lio_count}, slam_toolbox: {self.slam_count}")

    def ground_truth_callback(self, msg: TFMessage):
        # The Gazebo Pose_V -> TFMessage bridge does not populate a real
        # sim-time header stamp (observed: every transform.header.stamp
        # comes through as 0.0), so the receive-time via this node's own
        # clock is used instead -- with use_sim_time:=true that clock is
        # correctly synced to the same sim clock LIO-SAM/SLAM Toolbox use,
        # so timestamps line up for association in ate_rpe.py.
        receive_t = stamp_to_sec(self.get_clock().now().to_msg())
        for transform in msg.transforms:
            # Model pose vectors are typically keyed by frame name matching
            # the model name (possibly with a trailing suffix depending on
            # bridge version) -- match loosely.
            if self.robot_model_name not in transform.child_frame_id:
                continue
            tr = transform.transform.translation
            rot = transform.transform.rotation
            self.gt_writer.writerow([receive_t, tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w])
            self.gt_count += 1

    def lio_sam_callback(self, msg: Odometry):
        t = stamp_to_sec(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.lio_writer.writerow([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.lio_count += 1

    def slam_sample_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        t = stamp_to_sec(transform.header.stamp)
        tr = transform.transform.translation
        rot = transform.transform.rotation
        self.slam_writer.writerow([t, tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w])
        self.slam_count += 1

    def destroy_node(self):
        self.gt_file.close()
        self.lio_file.close()
        self.slam_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
