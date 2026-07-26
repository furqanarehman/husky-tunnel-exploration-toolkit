#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np


def load_binary_xyzi(path):
    with path.open("rb") as stream:
        point_count = None

        while True:
            line = stream.readline()

            if not line:
                raise RuntimeError(f"Missing DATA line in {path}")

            if line.startswith(b"POINTS "):
                point_count = int(line.split()[1])

            if line.startswith(b"DATA "):
                if line.strip() != b"DATA binary":
                    raise RuntimeError(
                        f"Only binary PCD files are supported: {path}"
                    )
                break

        if point_count is None:
            raise RuntimeError(f"Missing POINTS entry in {path}")

        values = np.fromfile(
            stream,
            dtype="<f4",
            count=point_count * 4,
        )

    if values.size != point_count * 4:
        raise RuntimeError(f"Incomplete point data in {path}")

    points = values.reshape(point_count, 4)
    return points[np.isfinite(points[:, :3]).all(axis=1)]


def mark_disks(mask, x_indices, y_indices, radius):
    height, width = mask.shape

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue

            x = x_indices + dx
            y = y_indices + dy

            valid = (
                (x >= 0)
                & (x < width)
                & (y >= 0)
                & (y < height)
            )

            mask[y[valid], x[valid]] = True


def main():
    parser = argparse.ArgumentParser(
        description="Convert a LIO-SAM PCD map into a Nav2 map."
    )
    parser.add_argument("map_directory", type=Path)
    parser.add_argument(
        "--output-prefix",
        type=Path,
        required=True,
    )
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--min-z", type=float, default=-0.20)
    parser.add_argument("--max-z", type=float, default=1.00)
    parser.add_argument("--free-radius", type=float, default=1.40)
    parser.add_argument("--wall-thickness", type=float, default=0.15)
    parser.add_argument("--margin", type=float, default=1.00)
    args = parser.parse_args()

    cloud = load_binary_xyzi(
        args.map_directory / "GlobalMap.pcd"
    )
    trajectory = load_binary_xyzi(
        args.map_directory / "trajectory.pcd"
    )

    wall_points = cloud[
        (cloud[:, 2] >= args.min_z)
        & (cloud[:, 2] <= args.max_z)
    ]

    if wall_points.size == 0:
        raise RuntimeError("No points remain in the selected Z range")

    all_x = np.concatenate((wall_points[:, 0], trajectory[:, 0]))
    all_y = np.concatenate((wall_points[:, 1], trajectory[:, 1]))

    min_x = (
        math.floor(
            (all_x.min() - args.margin) / args.resolution
        )
        * args.resolution
    )
    min_y = (
        math.floor(
            (all_y.min() - args.margin) / args.resolution
        )
        * args.resolution
    )
    max_x = all_x.max() + args.margin
    max_y = all_y.max() + args.margin

    width = int(math.ceil((max_x - min_x) / args.resolution)) + 1
    height = int(math.ceil((max_y - min_y) / args.resolution)) + 1

    occupied = np.zeros((height, width), dtype=bool)
    free = np.zeros((height, width), dtype=bool)

    wall_x = np.floor(
        (wall_points[:, 0] - min_x) / args.resolution
    ).astype(np.int32)
    wall_y = np.floor(
        (wall_points[:, 1] - min_y) / args.resolution
    ).astype(np.int32)

    wall_radius = max(
        1,
        int(math.ceil(args.wall_thickness / args.resolution)),
    )
    mark_disks(occupied, wall_x, wall_y, wall_radius)

    trajectory_x = np.floor(
        (trajectory[:, 0] - min_x) / args.resolution
    ).astype(np.int32)
    trajectory_y = np.floor(
        (trajectory[:, 1] - min_y) / args.resolution
    ).astype(np.int32)

    free_radius = max(
        1,
        int(math.ceil(args.free_radius / args.resolution)),
    )
    mark_disks(free, trajectory_x, trajectory_y, free_radius)

    # ROS map convention:
    #   0   = occupied
    #   254 = free
    #   205 = unknown
    image = np.full((height, width), 205, dtype=np.uint8)
    image[free] = 254
    image[occupied] = 0

    output_prefix = args.output_prefix.expanduser()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    pgm_path = Path(f"{output_prefix}.pgm")
    yaml_path = Path(f"{output_prefix}.yaml")

    # PGM rows run top-to-bottom; map coordinates run bottom-to-top.
    pgm_image = np.flipud(image)

    with pgm_path.open("wb") as stream:
        stream.write(
            f"P5\n{width} {height}\n255\n".encode("ascii")
        )
        stream.write(pgm_image.tobytes())

    yaml_path.write_text(
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {args.resolution:.6f}\n"
        f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )

    print(f"Input cloud points: {cloud.shape[0]}")
    print(f"Selected wall points: {wall_points.shape[0]}")
    print(f"Trajectory points: {trajectory.shape[0]}")
    print(f"Grid: {width} x {height}")
    print(f"Resolution: {args.resolution:.3f} m/cell")
    print(f"Origin: [{min_x:.3f}, {min_y:.3f}, 0.0]")
    print(f"Saved: {pgm_path}")
    print(f"Saved: {yaml_path}")


if __name__ == "__main__":
    main()
