#!/usr/bin/env python3
"""
map_completeness.py

Compares a comparison map against a reference map (both in standard ROS
map_server .pgm + .yaml format, e.g. saved via
`ros2 run nav2_map_server map_saver_cli`) and reports how complete the
comparison map is relative to the reference.

Since true CAD-level ground truth isn't available for this world (it's
built mostly from external Gazebo Fuel models we can't parse offline),
the reference map should be your most thorough / fully-explored run --
this gives a practical "how much of the best-known map does this run
cover" completeness measure, the same substitute commonly used in
real-world SLAM evaluation when perfect ground truth doesn't exist.

Reports:
  - Free-space completeness: % of reference's free cells that are also
    known (free or occupied) in the comparison map
  - Occupied/wall completeness: % of reference's occupied cells that are
    also occupied in the comparison map
  - Overall known-cell completeness: % of reference's known cells
    (free + occupied) that are also known in the comparison map

Usage:
    python3 map_completeness.py reference.yaml comparison.yaml
"""

import argparse
import os

import numpy as np
import yaml


def load_pgm(path):
    """Minimal PGM (P5, binary) parser -- avoids a PIL dependency."""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Unsupported PGM format in {path}: {magic}")

        def read_non_comment_line():
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            return line

        dims = read_non_comment_line().split()
        while len(dims) < 2:
            dims += read_non_comment_line().split()
        width, height = int(dims[0]), int(dims[1])

        maxval_line = read_non_comment_line()
        maxval = int(maxval_line.strip())

        dtype = np.uint8 if maxval < 256 else np.uint16
        data = np.frombuffer(f.read(width * height * dtype().itemsize), dtype=dtype)
        return data.reshape((height, width))


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    img = load_pgm(image_path)

    negate = meta.get("negate", 0)
    occupied_thresh = meta.get("occupied_thresh", 0.65)
    free_thresh = meta.get("free_thresh", 0.196)

    # Standard ROS map_server convention: pixel value -> occupancy
    # probability = (255 - pixel) / 255 (or pixel/255 if negate=1).
    if negate:
        occ_prob = img.astype(np.float64) / 255.0
    else:
        occ_prob = (255.0 - img.astype(np.float64)) / 255.0

    occupied = occ_prob > occupied_thresh
    free = occ_prob < free_thresh
    unknown = ~occupied & ~free

    return {
        "occupied": occupied,
        "free": free,
        "unknown": unknown,
        "resolution": meta["resolution"],
        "origin": meta["origin"],
        "shape": img.shape,
    }


def align_and_crop(ref, cmp_):
    """
    Both maps are expected to share the same resolution and world origin
    (typical when comparing runs on the same SLAM session/world), just
    possibly different pixel dimensions (map grows as SLAM explores).
    Crop/pad both to a common grid in world coordinates so cells line up.
    """
    if abs(ref["resolution"] - cmp_["resolution"]) > 1e-6:
        raise ValueError(
            f"Resolution mismatch: reference={ref['resolution']}, "
            f"comparison={cmp_['resolution']}. Maps must use the same "
            f"SLAM resolution to compare directly.")

    res = ref["resolution"]
    ref_origin = ref["origin"]
    cmp_origin = cmp_["origin"]

    # World-space bounding box covering both maps.
    ref_h, ref_w = ref["shape"]
    cmp_h, cmp_w = cmp_["shape"]

    ref_x0, ref_y0 = ref_origin[0], ref_origin[1]
    ref_x1, ref_y1 = ref_x0 + ref_w * res, ref_y0 + ref_h * res
    cmp_x0, cmp_y0 = cmp_origin[0], cmp_origin[1]
    cmp_x1, cmp_y1 = cmp_x0 + cmp_w * res, cmp_y0 + cmp_h * res

    world_x0 = min(ref_x0, cmp_x0)
    world_y0 = min(ref_y0, cmp_y0)
    world_x1 = max(ref_x1, cmp_x1)
    world_y1 = max(ref_y1, cmp_y1)

    common_w = int(round((world_x1 - world_x0) / res))
    common_h = int(round((world_y1 - world_y0) / res))

    def place(grid_dict, common_w, common_h, world_x0, world_y0, res):
        out_occupied = np.zeros((common_h, common_w), dtype=bool)
        out_free = np.zeros((common_h, common_w), dtype=bool)
        h, w = grid_dict["shape"]
        ox, oy = grid_dict["origin"][0], grid_dict["origin"][1]
        col_off = int(round((ox - world_x0) / res))
        row_off = int(round((oy - world_y0) / res))
        out_occupied[row_off:row_off + h, col_off:col_off + w] = grid_dict["occupied"]
        out_free[row_off:row_off + h, col_off:col_off + w] = grid_dict["free"]
        return out_occupied, out_free

    ref_occ, ref_free = place(ref, common_w, common_h, world_x0, world_y0, res)
    cmp_occ, cmp_free = place(cmp_, common_w, common_h, world_x0, world_y0, res)

    return ref_occ, ref_free, cmp_occ, cmp_free


def dilate(mask, radius_cells):
    """
    Simple binary dilation: grow True cells outward by radius_cells (in
    all directions), using a square structuring element. Pure numpy, no
    scipy dependency. Fine for the sparse occupied-cell masks involved
    here (walls are a small fraction of total map area).
    """
    if radius_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    out = mask.copy()
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx == 0 and dy == 0:
                continue
            shifted = np.zeros_like(mask)
            src_y0, src_y1 = max(0, -dy), min(h, h - dy)
            src_x0, src_x1 = max(0, -dx), min(w, w - dx)
            dst_y0, dst_y1 = max(0, dy), min(h, h + dy)
            dst_x0, dst_x1 = max(0, dx), min(w, w + dx)
            shifted[dst_y0:dst_y1, dst_x0:dst_x1] = mask[src_y0:src_y1, src_x0:src_x1]
            out |= shifted
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference_yaml", help="Reference (most complete) map .yaml")
    parser.add_argument("comparison_yaml", help="Comparison map .yaml to evaluate")
    parser.add_argument("--wall-tolerance-m", type=float, default=0.3,
                         help="Tolerance radius in metres for wall-completeness "
                              "matching, to absorb session-to-session SLAM "
                              "registration noise on thin walls (default 0.3)")
    args = parser.parse_args()

    ref = load_map(args.reference_yaml)
    cmp_ = load_map(args.comparison_yaml)

    print(f"Reference map:  {ref['shape']} px, resolution={ref['resolution']} m/px")
    print(f"Comparison map: {cmp_['shape']} px, resolution={cmp_['resolution']} m/px")

    ref_occ, ref_free, cmp_occ, cmp_free = align_and_crop(ref, cmp_)

    ref_known = ref_occ | ref_free
    cmp_known = cmp_occ | cmp_free

    ref_known_count = ref_known.sum()
    ref_occ_count = ref_occ.sum()
    ref_free_count = ref_free.sum()

    if ref_known_count == 0:
        print("Reference map has no known cells -- can't compute completeness.")
        return

    overall_completeness = (ref_known & cmp_known).sum() / ref_known_count * 100.0
    free_completeness = (
        (ref_free & cmp_known).sum() / ref_free_count * 100.0
        if ref_free_count > 0 else float("nan"))
    occupied_completeness_strict = (
        (ref_occ & cmp_occ).sum() / ref_occ_count * 100.0
        if ref_occ_count > 0 else float("nan"))

    # Tolerance-based wall completeness: reference and comparison maps
    # come from two INDEPENDENT SLAM sessions, each with its own
    # scan-matching/optimization -- even where both represent the exact
    # same physical wall, the actual occupied pixels can land a few cm
    # apart between sessions. That barely affects free-space overlap
    # (wide regions), but tunnel walls are only 1-2 px wide, so a strict
    # pixel-exact comparison can read as near-zero overlap even for a
    # wall that really was re-detected nearby. This checks "was a wall
    # detected NEAR here" instead of "at this exact pixel."
    tol_cells = int(round(args.wall_tolerance_m / ref["resolution"]))
    ref_occ_dilated = dilate(ref_occ, tol_cells)
    occupied_completeness_tolerant = (
        (ref_occ_dilated & cmp_occ).sum() / ref_occ_count * 100.0
        if ref_occ_count > 0 else float("nan"))

    print(f"\nReference known cells: {ref_known_count} "
          f"(free={ref_free_count}, occupied={ref_occ_count})")
    print(f"\nOverall known-cell completeness:  {overall_completeness:.1f}%")
    print(f"Free-space completeness:          {free_completeness:.1f}%")
    print(f"Occupied/wall completeness (strict, exact-pixel): "
          f"{occupied_completeness_strict:.1f}%")
    print(f"Occupied/wall completeness (tolerant, +/-{args.wall_tolerance_m}m): "
          f"{occupied_completeness_tolerant:.1f}%")
    print(f"\n(Use the tolerant number as the meaningful one when comparing "
          f"two independent SLAM sessions -- the strict number is easily "
          f"dominated by session-to-session registration noise on thin "
          f"wall lines and understates real completeness.)")


if __name__ == "__main__":
    main()
