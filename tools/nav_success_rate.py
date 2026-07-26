#!/usr/bin/env python3
"""
nav_success_rate.py

Parses a husky_tunnel_bringup exploration run log (the tee'd output of
`ros2 launch husky_tunnel_bringup tunnel_backtracking_exploration.launch.py`)
and computes navigation goal statistics:

  - total goals attempted, broken down by type (frontier / coverage
    fallback / return-home)
  - success rate overall and per type
  - average / median / max time-to-goal for successful goals
  - failure breakdown by Nav2 result code

Also writes a per-goal CSV for further analysis or plotting.

Usage:
    python3 nav_success_rate.py path/to/run_log.txt [--csv output.csv]

Notes on matching:
  - The explorer only runs one goal at a time, so goal-dispatch and
    goal-result lines can be paired up in the order they appear in the
    log -- no goal IDs are needed.
  - Nav2/rclcpp_action result codes vary a bit by ROS distro; this script
    reports the raw numeric code alongside a best-effort name so you can
    verify against your own build if needed, rather than asserting a
    single fixed mapping.
"""

import argparse
import csv
import re
import statistics
import sys

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
TS_RE = re.compile(r"\[(?:INFO|WARN|ERROR)\] \[(\d+\.\d+)\]")

DISPATCH_FRONTIER_RE = re.compile(
    r"Navigating to nearest reachable frontier: "
    r"goal=\(([-\d.]+), ([-\d.]+)\), path=([\d.]+) m, cells=(\d+)")
DISPATCH_COVERAGE_RE = re.compile(
    r"No frontier remains; navigating to nearest unvisited region: "
    r"goal=\(([-\d.]+), ([-\d.]+)\), path=([\d.]+) m, cells=(\d+)")
DISPATCH_HOME_RE = re.compile(
    r"Returning to recorded home pose: \(([-\d.]+), ([-\d.]+)\)")

RESULT_FRONTIER_SUCCESS_RE = re.compile(
    r"Frontier reached; scanning the complete map for the next branch")
RESULT_FRONTIER_FAIL_RE = re.compile(
    r"Frontier navigation ended with result code (\d+); "
    r"selecting another reachable branch")
RESULT_HOME_FAIL_RE = re.compile(
    r"Return-home navigation ended with result code (\d+); "
    r"retrying after map checks")
RESULT_HOME_SUCCESS_RE = re.compile(
    r"Home pose reached; autonomous mission finished")

# Best-effort names -- action_msgs/msg/GoalStatus terminal codes match what
# we've observed in practice (e.g. 6 = ABORTED, commonly seen when a goal
# is preempted by a new one). Verify against your own rclcpp_action version
# if precise semantics matter for your report.
RESULT_CODE_NAMES = {
    0: "UNKNOWN",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


def parse_log(path):
    goals = []  # list of dicts: type, start_ts, end_ts, success, result_code, x, y, path_m, cells
    pending = None

    with open(path, "r", errors="replace") as f:
        for raw_line in f:
            line = ANSI_RE.sub("", raw_line)
            ts_match = TS_RE.search(line)
            ts = float(ts_match.group(1)) if ts_match else None

            m = DISPATCH_FRONTIER_RE.search(line)
            if m and ts is not None:
                if pending:
                    goals.append(pending)
                pending = {
                    "type": "frontier", "start_ts": ts, "end_ts": None,
                    "success": None, "result_code": None,
                    "x": float(m.group(1)), "y": float(m.group(2)),
                    "path_m": float(m.group(3)), "cells": int(m.group(4)),
                }
                continue

            m = DISPATCH_COVERAGE_RE.search(line)
            if m and ts is not None:
                if pending:
                    goals.append(pending)
                pending = {
                    "type": "coverage_fallback", "start_ts": ts, "end_ts": None,
                    "success": None, "result_code": None,
                    "x": float(m.group(1)), "y": float(m.group(2)),
                    "path_m": float(m.group(3)), "cells": int(m.group(4)),
                }
                continue

            m = DISPATCH_HOME_RE.search(line)
            if m and ts is not None:
                if pending:
                    goals.append(pending)
                pending = {
                    "type": "return_home", "start_ts": ts, "end_ts": None,
                    "success": None, "result_code": None,
                    "x": float(m.group(1)), "y": float(m.group(2)),
                    "path_m": None, "cells": None,
                }
                continue

            if RESULT_FRONTIER_SUCCESS_RE.search(line) and ts is not None:
                if pending and pending["type"] in ("frontier", "coverage_fallback"):
                    pending["end_ts"] = ts
                    pending["success"] = True
                    pending["result_code"] = 4
                    goals.append(pending)
                    pending = None
                continue

            m = RESULT_FRONTIER_FAIL_RE.search(line)
            if m and ts is not None:
                if pending and pending["type"] in ("frontier", "coverage_fallback"):
                    pending["end_ts"] = ts
                    pending["success"] = False
                    pending["result_code"] = int(m.group(1))
                    goals.append(pending)
                    pending = None
                continue

            m = RESULT_HOME_FAIL_RE.search(line)
            if m and ts is not None:
                if pending and pending["type"] == "return_home":
                    pending["end_ts"] = ts
                    pending["success"] = False
                    pending["result_code"] = int(m.group(1))
                    goals.append(pending)
                    pending = None
                continue

            if RESULT_HOME_SUCCESS_RE.search(line) and ts is not None:
                if pending and pending["type"] == "return_home":
                    pending["end_ts"] = ts
                    pending["success"] = True
                    pending["result_code"] = 4
                    goals.append(pending)
                    pending = None
                continue

    if pending:
        # Run ended (log cut off / still in progress) with a goal in flight.
        pending["success"] = None
        goals.append(pending)

    return goals


def summarize(goals):
    total = len(goals)
    completed = [g for g in goals if g["success"] is not None]
    in_flight = total - len(completed)
    succeeded = [g for g in completed if g["success"]]
    failed = [g for g in completed if not g["success"]]

    print(f"Total goals attempted: {total}")
    if in_flight:
        print(f"  (of which {in_flight} still in-flight when the log ended)")
    print(f"Completed: {len(completed)}")
    if completed:
        rate = 100.0 * len(succeeded) / len(completed)
        print(f"Succeeded: {len(succeeded)}  Failed: {len(failed)}  "
              f"Success rate: {rate:.1f}%")

    print("\nBy goal type:")
    for goal_type in ("frontier", "coverage_fallback", "return_home"):
        subset = [g for g in completed if g["type"] == goal_type]
        if not subset:
            continue
        sub_succeeded = [g for g in subset if g["success"]]
        rate = 100.0 * len(sub_succeeded) / len(subset)
        print(f"  {goal_type:18s} attempted={len(subset):4d}  "
              f"succeeded={len(sub_succeeded):4d}  success_rate={rate:5.1f}%")

    durations = [g["end_ts"] - g["start_ts"] for g in succeeded
                 if g["end_ts"] is not None]
    if durations:
        print(f"\nTime-to-goal for successful goals (s):")
        print(f"  mean={statistics.mean(durations):.1f}  "
              f"median={statistics.median(durations):.1f}  "
              f"max={max(durations):.1f}  min={min(durations):.1f}")

    print("\nFailure breakdown by Nav2 result code:")
    code_counts = {}
    for g in failed:
        code = g["result_code"]
        code_counts[code] = code_counts.get(code, 0) + 1
    for code, count in sorted(code_counts.items(), key=lambda kv: -kv[1]):
        name = RESULT_CODE_NAMES.get(code, "?")
        print(f"  code {code} ({name}): {count}")


def write_csv(goals, out_path):
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "type", "start_ts", "end_ts", "duration_s", "success",
            "result_code", "x", "y", "path_m", "cells",
        ])
        for g in goals:
            duration = (g["end_ts"] - g["start_ts"]) if g["end_ts"] is not None else ""
            writer.writerow([
                g["type"], g["start_ts"], g["end_ts"] or "", duration,
                g["success"], g["result_code"] or "", g["x"], g["y"],
                g["path_m"] or "", g["cells"] or "",
            ])
    print(f"\nPer-goal CSV written to: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", help="Path to the exploration run log")
    parser.add_argument("--csv", default=None, help="Optional path to write a per-goal CSV")
    args = parser.parse_args()

    goals = parse_log(args.log_path)
    if not goals:
        print("No navigation goals found in this log. Wrong file, or the "
              "explorer never started sending goals.", file=sys.stderr)
        sys.exit(1)

    summarize(goals)
    if args.csv:
        write_csv(goals, args.csv)


if __name__ == "__main__":
    main()
