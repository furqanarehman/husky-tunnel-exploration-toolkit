#!/usr/bin/env python3

import math
import time
from collections import deque

import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


NEIGHBORS = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),            (0, 1),
    (1, -1),  (1, 0),  (1, 1),
]


class FrontierExplorer:

    def __init__(self, navigator):
        self.navigator = navigator
        self.latest_map = None
        self.blacklist = []
        self.last_no_frontier_report = 0.0

        self.navigator.declare_parameter("minimum_frontier_cells", 8)
        self.navigator.declare_parameter("minimum_goal_distance", 0.75)
        self.navigator.declare_parameter("maximum_goal_distance", 25.0)
        self.navigator.declare_parameter("exploration_step_distance", 3.0)
        self.navigator.declare_parameter("goal_standoff", 1.0)
        self.navigator.declare_parameter("goal_clearance", 0.70)
        self.navigator.declare_parameter("goal_search_radius", 1.2)
        self.navigator.declare_parameter("blacklist_radius", 1.2)
        self.navigator.declare_parameter("goal_timeout", 35.0)
        self.navigator.declare_parameter("maximum_goals", 0)
        self.navigator.declare_parameter("information_gain_weight", 0.15)
        self.navigator.declare_parameter("distance_score_weight", 1.0)
        self.navigator.declare_parameter("maximum_frontier_samples", 40)
        self.navigator.declare_parameter("free_threshold", 20)
        self.navigator.declare_parameter("dry_run", True)

        self.minimum_frontier_cells = self._integer_parameter(
            "minimum_frontier_cells"
        )
        self.minimum_goal_distance = self._double_parameter(
            "minimum_goal_distance"
        )
        self.maximum_goal_distance = self._double_parameter(
            "maximum_goal_distance"
        )
        self.exploration_step_distance = self._double_parameter(
            "exploration_step_distance"
        )
        self.goal_standoff = self._double_parameter("goal_standoff")
        self.goal_clearance = self._double_parameter("goal_clearance")
        self.goal_search_radius = self._double_parameter(
            "goal_search_radius"
        )
        self.blacklist_radius = self._double_parameter("blacklist_radius")
        self.goal_timeout = self._double_parameter("goal_timeout")
        self.maximum_goals = self._integer_parameter("maximum_goals")
        self.information_gain_weight = self._double_parameter(
            "information_gain_weight"
        )
        self.distance_score_weight = self._double_parameter(
            "distance_score_weight"
        )
        self.maximum_frontier_samples = self._integer_parameter(
            "maximum_frontier_samples"
        )
        self.free_threshold = self._integer_parameter("free_threshold")
        self.dry_run = self._boolean_parameter("dry_run")

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.navigator.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            map_qos,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self.navigator,
            spin_thread=False,
        )

    def _integer_parameter(self, name):
        return (
            self.navigator.get_parameter(name)
            .get_parameter_value()
            .integer_value
        )

    def _double_parameter(self, name):
        return (
            self.navigator.get_parameter(name)
            .get_parameter_value()
            .double_value
        )

    def _boolean_parameter(self, name):
        return (
            self.navigator.get_parameter(name)
            .get_parameter_value()
            .bool_value
        )

    def _map_callback(self, message):
        self.latest_map = message

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as error:
            self.navigator.get_logger().warning(
                f"Waiting for map to base_link transform: {error}"
            )
            return None

        translation = transform.transform.translation
        return translation.x, translation.y

    @staticmethod
    def _shifted_unknown(unknown, dy, dx):
        shifted = np.roll(unknown, shift=(dy, dx), axis=(0, 1))

        if dy > 0:
            shifted[:dy, :] = False
        elif dy < 0:
            shifted[dy:, :] = False

        if dx > 0:
            shifted[:, :dx] = False
        elif dx < 0:
            shifted[:, dx:] = False

        return shifted

    def frontier_clusters(self, grid):
        free = (grid >= 0) & (grid <= self.free_threshold)
        unknown = grid < 0
        touches_unknown = np.zeros(grid.shape, dtype=bool)

        for dy, dx in NEIGHBORS:
            touches_unknown |= self._shifted_unknown(unknown, dy, dx)

        frontier = free & touches_unknown
        visited = np.zeros(frontier.shape, dtype=bool)
        clusters = []
        height, width = frontier.shape

        for row, col in np.argwhere(frontier):
            if visited[row, col]:
                continue

            queue = deque([(int(row), int(col))])
            visited[row, col] = True
            cluster = []

            while queue:
                current_row, current_col = queue.popleft()
                cluster.append((current_row, current_col))

                for dy, dx in NEIGHBORS:
                    next_row = current_row + dy
                    next_col = current_col + dx

                    if not (0 <= next_row < height and 0 <= next_col < width):
                        continue
                    if visited[next_row, next_col]:
                        continue
                    if not frontier[next_row, next_col]:
                        continue

                    visited[next_row, next_col] = True
                    queue.append((next_row, next_col))

            if len(cluster) >= self.minimum_frontier_cells:
                clusters.append(np.asarray(cluster, dtype=np.int32))

        return clusters

    @staticmethod
    def origin_yaw(message):
        orientation = message.info.origin.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    def grid_to_world(self, row, col, message):
        resolution = message.info.resolution
        local_x = (float(col) + 0.5) * resolution
        local_y = (float(row) + 0.5) * resolution
        yaw = self.origin_yaw(message)
        origin = message.info.origin.position

        world_x = (
            origin.x
            + math.cos(yaw) * local_x
            - math.sin(yaw) * local_y
        )
        world_y = (
            origin.y
            + math.sin(yaw) * local_x
            + math.cos(yaw) * local_y
        )
        return world_x, world_y

    def world_to_grid(self, world_x, world_y, message):
        origin = message.info.origin.position
        yaw = self.origin_yaw(message)
        delta_x = world_x - origin.x
        delta_y = world_y - origin.y
        local_x = math.cos(yaw) * delta_x + math.sin(yaw) * delta_y
        local_y = -math.sin(yaw) * delta_x + math.cos(yaw) * delta_y
        resolution = message.info.resolution
        return local_y / resolution - 0.5, local_x / resolution - 0.5

    def _cell_is_clear(self, grid, row, col, radius):
        height, width = grid.shape
        row_min = max(0, row - radius)
        row_max = min(height, row + radius + 1)
        col_min = max(0, col - radius)
        col_max = min(width, col + radius + 1)
        window = grid[row_min:row_max, col_min:col_max]

        if window.size == 0:
            return False
        if np.any(window > self.free_threshold):
            return False
        if np.any(window < 0):
            return False
        return True

    def safe_goal_cell(self, grid, desired_row, desired_col, message):
        resolution = message.info.resolution
        search_cells = max(
            1,
            int(math.ceil(self.goal_search_radius / resolution)),
        )
        clearance_cells = max(
            1,
            int(math.ceil(self.goal_clearance / resolution)),
        )
        center_row = int(round(desired_row))
        center_col = int(round(desired_col))
        candidates = []

        for row_offset in range(-search_cells, search_cells + 1):
            for col_offset in range(-search_cells, search_cells + 1):
                row = center_row + row_offset
                col = center_col + col_offset

                if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
                    continue
                if not (0 <= grid[row, col] <= self.free_threshold):
                    continue

                distance_squared = (
                    (float(row) - desired_row) ** 2
                    + (float(col) - desired_col) ** 2
                )
                candidates.append((distance_squared, row, col))

        candidates.sort()

        for _, row, col in candidates:
            if self._cell_is_clear(
                grid,
                row,
                col,
                clearance_cells,
            ):
                return row, col

        return None

    def is_blacklisted(self, x, y):
        return any(
            math.hypot(x - blocked_x, y - blocked_y)
            < self.blacklist_radius
            for blocked_x, blocked_y in self.blacklist
        )

    def blacklist_goal(self, x, y):
        self.blacklist.append((x, y))
        self.blacklist = self.blacklist[-50:]

    def select_frontier(self, robot_x, robot_y):
        message = self.latest_map
        grid = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height,
            message.info.width,
        )
        robot_row, robot_col = self.world_to_grid(
            robot_x,
            robot_y,
            message,
        )
        best = None
        rejected = {
            "clusters": 0,
            "samples": 0,
            "unsafe": 0,
            "too_close": 0,
            "too_far": 0,
            "blacklisted": 0,
        }

        for cluster in self.frontier_clusters(grid):
            rejected["clusters"] += 1
            distance_squared = (
                (cluster[:, 0].astype(float) - robot_row) ** 2
                + (cluster[:, 1].astype(float) - robot_col) ** 2
            )
            farthest_first = np.argsort(distance_squared)[::-1]
            sample_count = min(
                len(farthest_first),
                self.maximum_frontier_samples,
            )

            # A frontier surrounding a locally observed area is often one
            # connected ring. Its centroid lies beside the robot and is not a
            # useful exploration target. Evaluate the farthest boundary cells
            # instead and keep the safest, most distant reachable candidate.
            for sample_index in farthest_first[:sample_count]:
                rejected["samples"] += 1
                frontier_row = float(cluster[sample_index, 0])
                frontier_col = float(cluster[sample_index, 1])
                vector_row = robot_row - frontier_row
                vector_col = robot_col - frontier_col
                vector_length = math.hypot(vector_row, vector_col)

                if vector_length < 1.0e-6:
                    continue

                standoff_cells = (
                    self.goal_standoff / message.info.resolution
                )
                available_travel = max(
                    0.0,
                    vector_length - standoff_cells,
                )
                travel_cells = min(
                    available_travel,
                    self.exploration_step_distance
                    / message.info.resolution,
                )
                desired_row = (
                    robot_row
                    - vector_row / vector_length * travel_cells
                )
                desired_col = (
                    robot_col
                    - vector_col / vector_length * travel_cells
                )
                goal_cell = self.safe_goal_cell(
                    grid,
                    desired_row,
                    desired_col,
                    message,
                )

                if goal_cell is None:
                    rejected["unsafe"] += 1
                    continue

                goal_row, goal_col = goal_cell
                goal_x, goal_y = self.grid_to_world(
                    goal_row,
                    goal_col,
                    message,
                )
                distance = math.hypot(
                    goal_x - robot_x,
                    goal_y - robot_y,
                )

                if distance < self.minimum_goal_distance:
                    rejected["too_close"] += 1
                    continue
                if distance > self.maximum_goal_distance:
                    rejected["too_far"] += 1
                    continue
                if self.is_blacklisted(goal_x, goal_y):
                    rejected["blacklisted"] += 1
                    continue

                frontier_x, frontier_y = self.grid_to_world(
                    frontier_row,
                    frontier_col,
                    message,
                )
                yaw = math.atan2(
                    frontier_y - goal_y,
                    frontier_x - goal_x,
                )
                information_gain = (
                    len(cluster) * message.info.resolution
                )
                score = -(
                    self.distance_score_weight * distance
                    + self.information_gain_weight * information_gain
                )

                if best is None or score < best[0]:
                    best = (
                        score,
                        goal_x,
                        goal_y,
                        yaw,
                        len(cluster),
                        distance,
                    )

        self.last_selection_summary = ", ".join(
            f"{name}={count}" for name, count in rejected.items()
        )
        return best

    def create_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def wait_for_inputs(self):
        self.navigator.get_logger().info(
            "Waiting for Nav2, the live map, and the robot transform..."
        )
        self.navigator._waitForNodeToActivate("bt_navigator")

        while rclpy.ok():
            rclpy.spin_once(self.navigator, timeout_sec=0.2)
            if self.latest_map is not None and self.robot_pose() is not None:
                return

    def explore(self):
        self.wait_for_inputs()
        attempted_goals = 0
        self.navigator.get_logger().info(
            "Autonomous frontier exploration started"
        )

        while rclpy.ok():
            rclpy.spin_once(self.navigator, timeout_sec=0.2)
            robot = self.robot_pose()

            if robot is None or self.latest_map is None:
                continue

            selected = self.select_frontier(*robot)

            if selected is None:
                now = time.monotonic()
                if now - self.last_no_frontier_report >= 5.0:
                    self.navigator.get_logger().info(
                        "No reachable frontier is currently available "
                        f"({self.last_selection_summary})"
                    )
                    self.last_no_frontier_report = now
                continue

            _, goal_x, goal_y, yaw, cells, distance = selected
            self.navigator.get_logger().info(
                f"Exploring frontier at x={goal_x:.2f}, y={goal_y:.2f}; "
                f"distance={distance:.2f} m, cells={cells}"
            )

            if self.dry_run:
                self.navigator.get_logger().info(
                    "Dry-run mode is enabled; frontier goal not sent"
                )
                dry_run_deadline = time.monotonic() + 5.0
                while rclpy.ok() and time.monotonic() < dry_run_deadline:
                    rclpy.spin_once(self.navigator, timeout_sec=0.1)
                continue

            accepted = self.navigator.goToPose(
                self.create_goal(goal_x, goal_y, yaw)
            )

            if not accepted:
                self.navigator.get_logger().warning(
                    "Frontier goal was rejected; blacklisting it"
                )
                self.blacklist_goal(goal_x, goal_y)
                continue

            started = time.monotonic()
            last_report = 0.0
            timed_out = False

            while rclpy.ok() and not self.navigator.isTaskComplete():
                now = time.monotonic()

                if now - started > self.goal_timeout:
                    self.navigator.get_logger().warning(
                        "Frontier goal timed out; canceling and blacklisting it"
                    )
                    self.navigator.cancelTask()
                    timed_out = True
                    break

                feedback = self.navigator.getFeedback()
                if feedback is not None and now - last_report >= 10.0:
                    self.navigator.get_logger().info(
                        f"Frontier goal: {feedback.distance_remaining:.2f} "
                        "m remaining"
                    )
                    last_report = now

            result = self.navigator.getResult()

            if not timed_out and result == TaskResult.SUCCEEDED:
                self.navigator.get_logger().info("Frontier goal reached")
            else:
                self.navigator.get_logger().warning(
                    "Frontier goal failed; selecting a different region"
                )
                self.blacklist_goal(goal_x, goal_y)

            attempted_goals += 1
            if (
                self.maximum_goals > 0
                and attempted_goals >= self.maximum_goals
            ):
                self.navigator.get_logger().info(
                    f"Finished the configured {attempted_goals} goal(s); "
                    "stopping exploration"
                )
                return

            # Allow the occupancy grid to incorporate the newly visible area
            # before choosing the next frontier.
            update_deadline = time.monotonic() + 4.0
            while rclpy.ok() and time.monotonic() < update_deadline:
                rclpy.spin_once(self.navigator, timeout_sec=0.1)


def main():
    rclpy.init()
    navigator = BasicNavigator(node_name="frontier_explorer")
    explorer = FrontierExplorer(navigator)

    try:
        explorer.explore()
    except KeyboardInterrupt:
        navigator.get_logger().info("Frontier exploration stopped")
        if rclpy.ok() and navigator.goal_handle is not None:
            navigator.cancelTask()
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
