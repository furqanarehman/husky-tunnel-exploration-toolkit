#!/usr/bin/env python3
"""
anomaly_detector.py

Geometric anomaly detection for the tunnel inspection project.

Approach
--------
The tunnel walls form two long, thin, continuous blobs of occupied cells
running the length of the corridor in the SLAM Toolbox occupancy grid
(/map). Anything else occupied -- compact, isolated, not touching the
outer wall boundary -- is very likely a real obstacle/anomaly, since
nothing else should exist in the free corridor space.

Pipeline per incoming /map message:
  1. Threshold the grid into occupied / free / unknown.
  2. Flood-fill label connected components of occupied cells.
  3. Classify each component as WALL (touches the grid border, or is
     highly elongated / very large) or an ANOMALY CANDIDATE (compact,
     within a reasonable size range, not touching the border).
  4. Track candidates across consecutive map updates by world position.
     A candidate must be seen in N consecutive updates before it is
     confirmed and published -- this filters out one-off SLAM noise
     and cells that are still settling as the map is being built.
  5. Publish confirmed anomalies as:
       - visualization_msgs/MarkerArray on ~/anomaly_markers (for RViz)
       - a simple JSON list of {x, y, cells} on ~/anomalies (std_msgs/String)

No scipy dependency -- connected-component labeling is a plain
BFS flood fill over a numpy array, consistent with the lightweight
style of the project's other scripts.
"""

import json
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class AnomalyDetector(Node):

    def __init__(self):
        super().__init__("anomaly_detector")

        # --- Parameters -----------------------------------------------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("occupied_threshold", 65)
        # Component classified as WALL (ignored) if it touches the grid
        # border, or its cell count exceeds wall_min_cells, or its
        # elongation (long side / short side of bounding box) exceeds
        # wall_min_elongation.
        self.declare_parameter("wall_min_cells", 400)
        self.declare_parameter("wall_min_elongation", 6.0)
        # Component considered a candidate anomaly only if its cell
        # count falls in [anomaly_min_cells, anomaly_max_cells].
        self.declare_parameter("anomaly_min_cells", 4)
        self.declare_parameter("anomaly_max_cells", 250)
        # A candidate must appear within this world-distance (metres)
        # of a previous detection, on this many consecutive map
        # updates, before it is confirmed and published.
        self.declare_parameter("match_radius_m", 0.75)
        self.declare_parameter("confirmations_required", 4)
        # Drop a tracked-but-unconfirmed candidate if it hasn't been
        # re-seen within this many map updates.
        self.declare_parameter("candidate_timeout_updates", 10)
        # Safety-net backup to the 8-connectivity fix: a candidate is
        # dropped if its centroid is within this distance (metres) of any
        # wall-classified component's bounding box. Expressed in metres
        # (not cells) so it stays correct regardless of map resolution.
        # Default is wide enough to clear the "ghost wall" SLAM drift
        # artifact seen in testing -- a faint duplicate wall line that can
        # appear ~0.4-0.5 m offset from the real wall.
        self.declare_parameter("wall_buffer_m", 0.7)
        # Connected-component labeling connects cells within this many
        # grid cells of each other (not just direct 8-neighbors). This
        # bridges small gaps in sparsely-mapped wall sections -- e.g. a
        # wall region the robot has only driven past once -- so the whole
        # wall merges into one long blob that correctly passes the
        # wall_min_cells / wall_min_elongation checks, instead of
        # fragmenting into many small pieces that each look like a
        # separate small "anomaly."
        self.declare_parameter("gap_bridge_radius_cells", 2)

        self.map_topic = self.get_parameter("map_topic").value
        self.occupied_threshold = self.get_parameter("occupied_threshold").value
        self.wall_min_cells = self.get_parameter("wall_min_cells").value
        self.wall_min_elongation = self.get_parameter("wall_min_elongation").value
        self.anomaly_min_cells = self.get_parameter("anomaly_min_cells").value
        self.anomaly_max_cells = self.get_parameter("anomaly_max_cells").value
        self.match_radius_m = self.get_parameter("match_radius_m").value
        self.confirmations_required = self.get_parameter("confirmations_required").value
        self.candidate_timeout_updates = self.get_parameter("candidate_timeout_updates").value
        self.wall_buffer_m = self.get_parameter("wall_buffer_m").value
        self.gap_bridge_radius_cells = self.get_parameter("gap_bridge_radius_cells").value

        r = self.gap_bridge_radius_cells
        self._neighbor_offsets = [
            (dx, dy)
            for dx in range(-r, r + 1)
            for dy in range(-r, r + 1)
            if not (dx == 0 and dy == 0)
        ]

        # --- State -------------------------------------------------------
        # Each tracked candidate: {"x": float, "y": float, "cells": int,
        #                          "hits": int, "last_seen_update": int,
        #                          "confirmed": bool}
        self.tracked = []
        self.update_count = 0
        self.confirmed_count = 0

        # --- I/O -----------------------------------------------------
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos)

        self.marker_pub = self.create_publisher(
            MarkerArray, "~/anomaly_markers", 10)
        self.anomaly_pub = self.create_publisher(
            String, "~/anomalies", 10)

        self.get_logger().info(
            f"Anomaly detector ready, listening on {self.map_topic}")

    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.update_count += 1

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        if width == 0 or height == 0:
            return

        grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
        occupied = grid >= self.occupied_threshold

        components = self._label_components(occupied)

        wall_bboxes = []
        raw_candidates = []
        for comp in components:
            classification = self._classify(comp, width, height)
            if classification == "wall":
                wall_bboxes.append(comp["bbox"])
            elif classification == "anomaly_candidate":
                raw_candidates.append(comp)

        candidates_this_update = []
        wall_buffer_cells = self.wall_buffer_m / resolution
        for comp in raw_candidates:
            cx_cell, cy_cell = comp["centroid"]
            if self._too_close_to_wall(cx_cell, cy_cell, wall_bboxes, wall_buffer_cells):
                continue
            world_x = origin_x + (cx_cell + 0.5) * resolution
            world_y = origin_y + (cy_cell + 0.5) * resolution
            candidates_this_update.append(
                {"x": world_x, "y": world_y, "cells": comp["size"]})

        self._update_tracks(candidates_this_update)
        self._publish()

    # ------------------------------------------------------------------
    def _label_components(self, occupied: np.ndarray):
        """Plain BFS flood-fill connected-component labeling (4-connectivity)."""
        height, width = occupied.shape
        visited = np.zeros_like(occupied, dtype=bool)
        components = []

        for start_y in range(height):
            for start_x in range(width):
                if not occupied[start_y, start_x] or visited[start_y, start_x]:
                    continue

                # BFS from this seed cell.
                q = deque([(start_x, start_y)])
                visited[start_y, start_x] = True
                cells = []
                min_x = max_x = start_x
                min_y = max_y = start_y
                touches_border = False

                while q:
                    x, y = q.popleft()
                    cells.append((x, y))
                    min_x = min(min_x, x)
                    max_x = max(max_x, x)
                    min_y = min(min_y, y)
                    max_y = max(max_y, y)
                    if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                        touches_border = True

                    # Gap-bridging connectivity: treat any occupied cell
                    # within gap_bridge_radius_cells as connected, not just
                    # direct neighbors. See parameter docstring above for
                    # why -- this merges sparsely-mapped wall sections into
                    # one long correctly-classified blob.
                    for dx, dy in self._neighbor_offsets:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if occupied[ny, nx] and not visited[ny, nx]:
                                visited[ny, nx] = True
                                q.append((nx, ny))

                sum_x = sum(c[0] for c in cells)
                sum_y = sum(c[1] for c in cells)
                n = len(cells)
                components.append({
                    "size": n,
                    "bbox": (min_x, min_y, max_x, max_y),
                    "touches_border": touches_border,
                    "centroid": (sum_x / n, sum_y / n),
                })

        return components

    # ------------------------------------------------------------------
    def _classify(self, comp, width, height):
        bx0, by0, bx1, by1 = comp["bbox"]
        w = (bx1 - bx0 + 1)
        h = (by1 - by0 + 1)
        long_side = max(w, h)
        short_side = max(1, min(w, h))
        elongation = long_side / short_side

        if comp["touches_border"]:
            return "wall"
        if comp["size"] >= self.wall_min_cells:
            return "wall"
        if elongation >= self.wall_min_elongation:
            return "wall"
        if self.anomaly_min_cells <= comp["size"] <= self.anomaly_max_cells:
            return "anomaly_candidate"
        return "ignore"

    # ------------------------------------------------------------------
    def _too_close_to_wall(self, px, py, wall_bboxes, buf):
        """True if point (px, py), in grid cells, lies within
        buf cells of any wall component's bounding box."""
        for (bx0, by0, bx1, by1) in wall_bboxes:
            # Distance from point to the (expanded) rectangle; 0 if inside.
            dx = max(bx0 - buf - px, 0, px - (bx1 + buf))
            dy = max(by0 - buf - py, 0, py - (by1 + buf))
            if dx == 0 and dy == 0:
                return True
        return False

    # ------------------------------------------------------------------
    def _update_tracks(self, candidates_this_update):
        matched_track_indices = set()

        for cand in candidates_this_update:
            best_idx = None
            best_dist = self.match_radius_m
            for idx, track in enumerate(self.tracked):
                if idx in matched_track_indices:
                    continue
                dist = math.hypot(cand["x"] - track["x"], cand["y"] - track["y"])
                if dist < best_dist:
                    best_dist = dist
                    best_idx = idx

            if best_idx is not None:
                track = self.tracked[best_idx]
                # Running average position, keeps the marker stable.
                track["x"] = (track["x"] + cand["x"]) / 2.0
                track["y"] = (track["y"] + cand["y"]) / 2.0
                track["cells"] = cand["cells"]
                track["hits"] += 1
                track["last_seen_update"] = self.update_count
                matched_track_indices.add(best_idx)

                if not track["confirmed"] and track["hits"] >= self.confirmations_required:
                    track["confirmed"] = True
                    self.confirmed_count += 1
                    self.get_logger().info(
                        f"Anomaly CONFIRMED #{self.confirmed_count} at "
                        f"({track['x']:.2f}, {track['y']:.2f}), "
                        f"~{track['cells']} occupied cells")
            else:
                self.tracked.append({
                    "x": cand["x"], "y": cand["y"], "cells": cand["cells"],
                    "hits": 1, "last_seen_update": self.update_count,
                    "confirmed": False,
                })

        # Drop stale, unconfirmed candidates that haven't been re-seen recently.
        self.tracked = [
            t for t in self.tracked
            if t["confirmed"] or
            (self.update_count - t["last_seen_update"]) <= self.candidate_timeout_updates
        ]

    # ------------------------------------------------------------------
    def _publish(self):
        confirmed = [t for t in self.tracked if t["confirmed"]]

        marker_array = MarkerArray()
        for i, t in enumerate(confirmed):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "anomalies"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = t["x"]
            marker.pose.position.y = t["y"]
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.9
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = marker.header.stamp
            text_marker.ns = "anomaly_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = t["x"]
            text_marker.pose.position.y = t["y"]
            text_marker.pose.position.z = 1.1
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.35
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"anomaly #{i}"
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

        payload = json.dumps([
            {"x": round(t["x"], 2), "y": round(t["y"], 2), "cells": t["cells"]}
            for t in confirmed
        ])
        self.anomaly_pub.publish(String(data=payload))


def main():
    rclpy.init()
    node = AnomalyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
