#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# One-way inspection route through the straight tunnel and around the corner.
# Coordinates are expressed in the saved map frame.
INSPECTION_ROUTE = [
    ("Straight section 1", 2.0, 0.0, 0.0),
    ("Straight section 2", 4.0, 0.0, 0.0),
    ("Straight section 3", 12.0, 0.0, 0.0),
    ("Straight section 4", 20.0, 0.0, 0.0),
    ("Before corner", 27.5, 0.0, 0.0),
    ("After corner", 29.0, 4.0, math.pi / 2.0),
    ("Vertical section 1", 29.0, 9.0, math.pi / 2.0),
    ("Vertical section 2", 29.0, 15.0, math.pi / 2.0),
]


def create_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.declare_parameter("waypoint_limit", 0)
    navigator.declare_parameter("start_waypoint", 1)
    navigator.declare_parameter("inspection_pause", 3.0)

    waypoint_limit = (
        navigator.get_parameter("waypoint_limit")
        .get_parameter_value()
        .integer_value
    )
    inspection_pause = (
        navigator.get_parameter("inspection_pause")
        .get_parameter_value()
        .double_value
    )

    start_waypoint = (
        navigator.get_parameter("start_waypoint")
        .get_parameter_value()
        .integer_value
    )
    start_index = max(0, start_waypoint - 1)

    route = INSPECTION_ROUTE[start_index:]
    if waypoint_limit > 0:
        route = route[:waypoint_limit]

    try:
        navigator.get_logger().info(
            "Waiting for localization and Navigation2..."
        )
        navigator._waitForNodeToActivate("amcl")
        navigator._waitForNodeToActivate("bt_navigator")
        navigator.get_logger().info("Nav2 is ready without resetting AMCL.")

        navigator.get_logger().info(
            f"Starting inspection mission with {len(route)} waypoint(s)."
        )

        for index, (name, x, y, yaw) in enumerate(route, start=1):
            goal = create_pose(navigator, x, y, yaw)

            navigator.get_logger().info(
                f"Waypoint {index}/{len(route)}: {name} "
                f"at x={x:.1f}, y={y:.1f}"
            )

            navigator.goToPose(goal)
            last_report = 0.0

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                now = time.monotonic()

                if feedback is not None and now - last_report >= 5.0:
                    distance = getattr(
                        feedback,
                        "distance_remaining",
                        float("nan"),
                    )
                    navigator.get_logger().info(
                        f"{name}: {distance:.2f} m remaining"
                    )
                    last_report = now

                time.sleep(0.2)

            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info(
                    f"Reached {name}. Inspecting for "
                    f"{inspection_pause:.1f} seconds."
                )
                time.sleep(inspection_pause)
            elif result == TaskResult.CANCELED:
                navigator.get_logger().error(
                    f"Mission canceled at {name}."
                )
                return 2
            else:
                navigator.get_logger().error(
                    f"Navigation failed at {name}. Mission stopped."
                )
                return 1

        navigator.get_logger().info(
            "Inspection mission completed successfully."
        )
        return 0

    except KeyboardInterrupt:
        navigator.get_logger().warning("Mission interrupted; canceling active goal.")
        navigator.cancelTask()
        time.sleep(0.5)
        return 130

    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
